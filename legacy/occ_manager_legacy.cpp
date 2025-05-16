#include "config.h"

#include "legacy/occ_manager_legacy.hpp"

#include "i2c_occ.hpp"
#include "occ_dbus.hpp"
#include "occ_errors.hpp"
#include "utils.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>

namespace open_power
{
namespace occ
{

constexpr uint32_t fruTypeNotAvailable = 0xFF;
constexpr auto fruTypeSuffix = "fru_type";
constexpr auto faultSuffix = "fault";
constexpr auto inputSuffix = "input";
constexpr auto maxSuffix = "max";

const auto HOST_ON_FILE = "/run/openbmc/host@0-on";

using namespace phosphor::logging;
using namespace std::literals::chrono_literals;

template <typename T>
T readFile(const std::string& path)
{
    std::ifstream ifs;
    ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit |
                   std::ifstream::eofbit);
    T data;

    try
    {
        ifs.open(path);
        ifs >> data;
        ifs.close();
    }
    catch (const std::exception& e)
    {
        auto err = errno;
        throw std::system_error(err, std::generic_category());
    }

    return data;
}

// findAndCreateObjects():
// Takes care of getting the required objects created and
// finds the available devices/processors.
// (function is called everytime the discoverTimer expires)
// - create the PowerMode object to control OCC modes
// - create statusObjects for each OCC device found
// - waits for OCC Active sensors PDRs to become available
// - restart discoverTimer if all data is not available yet
void Manager::findAndCreateObjects()
{
#ifdef I2C_OCC
    // Make sure we have a valid path string
    static_assert(sizeof(DEV_PATH) != 0);

    auto deviceNames = i2c_occ::getOccHwmonDevices(DEV_PATH);
    for (auto& name : deviceNames)
    {
        i2c_occ::i2cToDbus(name);
        name = std::string(OCC_NAME) + '_' + name;
        auto path = fs::path(OCC_CONTROL_ROOT) / name;
        statusObjects.emplace_back(
            std::make_unique<Status>(event, path.c_str(), *this));
    }
    // The first device is master occ
    pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
        *statusObjects.front());

#else  // NOT I2C_OCC
    for (auto id = 0; id < MAX_CPUS; ++id)
    {
        // Create one occ per cpu
        auto occ = std::string(OCC_NAME) + std::to_string(id);
        createObjects(occ);
    }
#endif // I2C_OCC
}

std::vector<int> Manager::findOCCsInDev()
{
    std::vector<int> occs;
    std::regex expr{R"(occ(\d+)$)"};

    for (auto& file : fs::directory_iterator("/dev"))
    {
        std::smatch match;
        std::string path{file.path().string()};
        if (std::regex_search(path, match, expr))
        {
            auto num = std::stoi(match[1].str());

            // /dev numbering starts at 1, ours starts at 0.
            occs.push_back(num - 1);
        }
    }

    return occs;
}

int Manager::cpuCreated(sdbusplus::message_t& msg)
{
    namespace fs = std::filesystem;

    sdbusplus::message::object_path o;
    msg.read(o);
    fs::path cpuPath(std::string(std::move(o)));

    auto name = cpuPath.filename().string();
    auto index = name.find(CPU_NAME);
    name.replace(index, std::strlen(CPU_NAME), OCC_NAME);

    createObjects(name);

    return 0;
}

void Manager::createObjects(const std::string& occ)
{
    auto path = fs::path(OCC_CONTROL_ROOT) / occ;

    statusObjects.emplace_back(std::make_unique<Status>(
        event, path.c_str(), *this,
        std::bind(std::mem_fn(&Manager::statusCallBack), this,
                  std::placeholders::_1, std::placeholders::_2)));

    // Create the power cap monitor object
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
            *statusObjects.back());
    }

    if (statusObjects.back()->isMasterOcc())
    {
        lg2::info("Manager::createObjects(): OCC{INST} is the master", "INST",
                  statusObjects.back()->getOccInstanceID());
        _pollTimer->setEnabled(false);

    }

    passThroughObjects.emplace_back(
        std::make_unique<PassThrough>(path.c_str()));
}

// If a reset is not already outstanding, set a flag to indicate that a reset is
// needed.
void Manager::resetOccRequest(instanceID instance)
{
    if (!resetRequired)
    {
        resetRequired = true;
        resetInstance = instance;
        lg2::error(
            "resetOccRequest: PM Complex reset was requested due to OCC{INST}",
            "INST", instance);
    }
    else if (instance != resetInstance)
    {
        lg2::warning(
            "resetOccRequest: Ignoring PM Complex reset request for OCC{INST}, because reset already outstanding for OCC{RINST}",
            "INST", instance, "RINST", resetInstance);
    }
}

// If a reset has not been started, initiate an OCC reset via PLDM
void Manager::initiateOccRequest(instanceID instance)
{
    if (!resetInProgress)
    {
        resetInProgress = true;
        resetInstance = instance;
        lg2::error(
            "initiateOccRequest: Initiating PM Complex reset due to OCC{INST}",
            "INST", instance);

        // Make sure ALL OCC comm stops to all OCCs before the reset
        for (auto& obj : statusObjects)
        {
            if (obj->occActive())
            {
                obj->occActive(false);
            }
        }
        resetRequired = false;
    }
    else
    {
        lg2::warning(
            "initiateOccRequest: Ignoring PM Complex reset request for OCC{INST}, because reset already in process for OCC{RINST}",
            "INST", instance, "RINST", resetInstance);
    }
}

void Manager::statusCallBack(instanceID instance, bool status)
{
    if (status == true)
    {
        if (resetInProgress)
        {
            lg2::info(
                "statusCallBack: Ignoring OCC{INST} activate because a reset has been initiated due to OCC{RINST}",
                "INST", instance, "RINST", resetInstance);
            return;
        }

        // OCC went active
        ++activeCount;

        if (activeCount == statusObjects.size())
        {
            // Verify master OCC and start presence monitor
            validateOccMaster();
        }

        // Start poll timer if not already started (since at least one OCC is
        // running)
        if (!_pollTimer->isEnabled())
        {
            // An OCC just went active, PM Complex is just coming online so
            // clear any outstanding reset requests
            if (resetRequired)
            {
                resetRequired = false;
                lg2::error(
                    "statusCallBack: clearing resetRequired (since OCC{INST} went active, resetInProgress={RIP})",
                    "INST", instance, "RIP", resetInProgress);
            }

            lg2::info("Manager: OCCs will be polled every {TIME} seconds",
                      "TIME", pollInterval);

            // Send poll and start OCC poll timer
            pollerTimerExpired();
        }
    }
    else
    {
        // OCC went away
        if (activeCount > 0)
        {
            --activeCount;
        }
        else
        {
            lg2::info("OCC{INST} disabled, and no other OCCs are active",
                      "INST", instance);
        }

        if (activeCount == 0)
        {
            // No OCCs are running

            if (resetInProgress)
            {
                // All OCC active sensors are clear (reset should be in
                // progress)
                lg2::info(
                    "statusCallBack: Clearing resetInProgress (activeCount={COUNT}, OCC{INST}, status={STATUS})",
                    "COUNT", activeCount, "INST", instance, "STATUS", status);
                resetInProgress = false;
                resetInstance = 255;
            }

            // Stop OCC poll timer
            if (_pollTimer->isEnabled())
            {
                lg2::info(
                    "Manager::statusCallBack(): OCCs are not running, stopping poll timer");
                _pollTimer->setEnabled(false);
            }
        }
        else if (resetInProgress)
        {
            lg2::info(
                "statusCallBack: Skipping clear of resetInProgress (activeCount={COUNT}, OCC{INST}, status={STATUS})",
                "COUNT", activeCount, "INST", instance, "STATUS", status);
        }
    }
}


void Manager::initTimerObjects()
{
    _pollTimer = std::make_unique<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>(
        sdpEvent, std::bind(&Manager::pollerTimerExpired, this));
}

void Manager::pollerTimerExpired()
{
    if (!_pollTimer)
    {
        lg2::error("pollerTimerExpired() ERROR: Timer not defined");
        return;
    }

    for (auto& obj : statusObjects)
    {
        if (!obj->occActive())
        {
            // OCC is not running yet
            continue;
        }

        // Read sysfs to force kernel to poll OCC
        obj->readOccState();
    }

    if (activeCount > 0)
    {
        // Restart OCC poll timer
        _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
    }
    else
    {
        // No OCCs running, so poll timer will not be restarted
        lg2::info(
            "Manager::pollerTimerExpired: poll timer will not be restarted");
    }
}

// Read the altitude from DBus
void Manager::readAltitude()
{
    static bool traceAltitudeErr = true;

    utils::PropertyValue altitudeProperty{};
    try
    {
        altitudeProperty = utils::getProperty(ALTITUDE_PATH, ALTITUDE_INTERFACE,
                                              ALTITUDE_PROP);
        auto sensorVal = std::get<double>(altitudeProperty);
        if (sensorVal < 0xFFFF)
        {
            if (sensorVal < 0)
            {
                altitude = 0;
            }
            else
            {
                // Round to nearest meter
                altitude = uint16_t(sensorVal + 0.5);
            }
            lg2::debug("readAltitude: sensor={VALUE} ({ALT}m)", "VALUE",
                       sensorVal, "ALT", altitude);
            traceAltitudeErr = true;
        }
        else
        {
            if (traceAltitudeErr)
            {
                traceAltitudeErr = false;
                lg2::debug("Invalid altitude value: {ALT}", "ALT", sensorVal);
            }
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        if (traceAltitudeErr)
        {
            traceAltitudeErr = false;
            lg2::info("Unable to read Altitude: {ERROR}", "ERROR", e.what());
        }
        altitude = 0xFFFF; // not available
    }
}

// Callback function when ambient temperature changes
void Manager::ambientCallback(sdbusplus::message_t& msg)
{
    double currentTemp = 0;
    uint8_t truncatedTemp = 0xFF;
    std::string msgSensor;
    std::map<std::string, std::variant<double>> msgData;
    msg.read(msgSensor, msgData);

    auto valPropMap = msgData.find(AMBIENT_PROP);
    if (valPropMap == msgData.end())
    {
        lg2::debug("ambientCallback: Unknown ambient property changed");
        return;
    }
    currentTemp = std::get<double>(valPropMap->second);
    if (std::isnan(currentTemp))
    {
        truncatedTemp = 0xFF;
    }
    else
    {
        if (currentTemp < 0)
        {
            truncatedTemp = 0;
        }
        else
        {
            // Round to nearest degree C
            truncatedTemp = uint8_t(currentTemp + 0.5);
        }
    }

    // If ambient changes, notify OCCs
    if (truncatedTemp != ambient)
    {
        lg2::debug("ambientCallback: Ambient change from {OLD} to {NEW}C",
                   "OLD", ambient, "NEW", currentTemp);

        ambient = truncatedTemp;
        if (altitude == 0xFFFF)
        {
            // No altitude yet, try reading again
            readAltitude();
        }

        lg2::debug("ambientCallback: Ambient: {TEMP}C, altitude: {ALT}m",
                   "TEMP", ambient, "ALT", altitude);
    }
}

// return the current ambient and altitude readings
void Manager::getAmbientData(bool& ambientValid, uint8_t& ambientTemp,
                             uint16_t& altitudeValue) const
{
    ambientValid = true;
    ambientTemp = ambient;
    altitudeValue = altitude;

    if (ambient == 0xFF)
    {
        ambientValid = false;
    }
}

// Verify single master OCC and start presence monitor
void Manager::validateOccMaster()
{
    int masterInstance = -1;
    for (auto& obj : statusObjects)
    {
        auto instance = obj->getOccInstanceID();

        if (obj->isMasterOcc())
        {
            obj->addPresenceWatchMaster();

            if (masterInstance == -1)
            {
                masterInstance = instance;
            }
            else
            {
                lg2::error(
                    "validateOccMaster: Multiple OCC masters! ({MAST1} and {MAST2})",
                    "MAST1", masterInstance, "MAST2", instance);
                // request reset
                obj->deviceError(Error::Descriptor(PRESENCE_ERROR_PATH));
            }
        }
    }

    if (masterInstance < 0)
    {
        lg2::error("validateOccMaster: Master OCC not found! (of {NUM} OCCs)",
                   "NUM", statusObjects.size());
        // request reset
        statusObjects.front()->deviceError(
            Error::Descriptor(PRESENCE_ERROR_PATH));
    }
    else
    {
        lg2::info("validateOccMaster: OCC{INST} is master of {COUNT} OCCs",
                  "INST", masterInstance, "COUNT", activeCount);
    }
}

void Manager::updatePcapBounds() const
{
    if (pcap)
    {
        pcap->updatePcapBounds();
    }
}

// Clean up any variables since the OCC is no longer running.
// Called when pldm receives an event indicating host is powered off.
void Manager::hostPoweredOff()
{
    if (resetRequired)
    {
        lg2::info("hostPoweredOff: Clearing resetRequired for OCC{INST}",
                  "INST", resetInstance);
        resetRequired = false;
    }
    if (resetInProgress)
    {
        lg2::info("hostPoweredOff: Clearing resetInProgress for OCC{INST}",
                  "INST", resetInstance);
        resetInProgress = false;
    }
    resetInstance = 255;
}

} // namespace occ
} // namespace open_power
