#include "config.h"

#include "occ_manager.hpp"

#include "i2c_occ.hpp"
#include "occ_dbus.hpp"
#include "utils.hpp"

#include <cmath>
#include <experimental/filesystem>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <regex>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

constexpr uint32_t fruTypeNotAvailable = 0xFF;

using namespace phosphor::logging;

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
    }
    catch (const std::exception& e)
    {
        auto err = errno;
        throw std::system_error(err, std::generic_category());
    }

    return data;
}

void Manager::findAndCreateObjects()
{
    for (auto id = 0; id < MAX_CPUS; ++id)
    {
        // Create one occ per cpu
        auto occ = std::string(OCC_NAME) + std::to_string(id);
        createObjects(occ);
    }
}

int Manager::cpuCreated(sdbusplus::message::message& msg)
{
    namespace fs = std::experimental::filesystem;

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

    passThroughObjects.emplace_back(
        std::make_unique<PassThrough>(path.c_str()));

    statusObjects.emplace_back(std::make_unique<Status>(
        event, path.c_str(), *this,
        std::bind(std::mem_fn(&Manager::statusCallBack), this,
                  std::placeholders::_1)
#ifdef PLDM
            ,
        std::bind(std::mem_fn(&pldm::Interface::resetOCC), pldmHandle.get(),
                  std::placeholders::_1)
#endif
            ));

    // Create the power cap monitor object for master occ (0)
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
            *statusObjects.front());
    }

#ifdef POWER10
    // Create the power mode monitor object for master occ (0)
    if (!pmode)
    {
        pmode = std::make_unique<open_power::occ::powermode::PowerMode>(
            *statusObjects.front());
    }
#endif
}

void Manager::statusCallBack(bool status)
{
    using InternalFailure =
        sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

    // At this time, it won't happen but keeping it
    // here just in case something changes in the future
    if ((activeCount == 0) && (!status))
    {
        log<level::ERR>("Invalid update on OCCActive");
        elog<InternalFailure>();
    }

    activeCount += status ? 1 : -1;

    // Only start presence detection if all the OCCs are bound
    if (activeCount == statusObjects.size())
    {
        for (auto& obj : statusObjects)
        {
            obj->addPresenceWatchMaster();
        }
    }

    if ((!_pollTimer->isEnabled()) && (activeCount > 0))
    {
        log<level::INFO>(fmt::format("Manager::statusCallBack(): {} OCCs will "
                                     "be polled every {} seconds",
                                     activeCount, pollInterval)
                             .c_str());

        // Send poll and start OCC poll timer
        pollerTimerExpired();
    }
    else if ((_pollTimer->isEnabled()) && (activeCount == 0))
    {
        // Stop OCC poll timer
        log<level::INFO>("Manager::statusCallBack(): OCCs are not running, "
                         "stopping poll timer");
        _pollTimer->setEnabled(false);

#ifdef READ_OCC_SENSORS
        for (auto& obj : statusObjects)
        {
            setSensorValueToNaN(obj->getOccInstanceID());
        }
#endif
    }
}

#ifdef I2C_OCC
void Manager::initStatusObjects()
{
    // Make sure we have a valid path string
    static_assert(sizeof(DEV_PATH) != 0);

    auto deviceNames = i2c_occ::getOccHwmonDevices(DEV_PATH);
    auto occMasterName = deviceNames.front();
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
        *statusObjects.front(), occMasterName);
#ifdef POWER10
    pmode = std::make_unique<open_power::occ::powermode::PowerMode>(
        *statusObjects.front());
#endif
}
#endif

#ifdef PLDM
bool Manager::updateOCCActive(instanceID instance, bool status)
{
    return (statusObjects[instance])->occActive(status);
}
#endif

void Manager::pollerTimerExpired()
{
    if (activeCount == 0)
    {
        // No OCCs running, so poll timer will not be restarted
        log<level::INFO>("Manager::pollerTimerExpire(): No OCCs running, poll "
                         "timer not restarted");
    }

    if (!_pollTimer)
    {
        log<level::ERR>(
            "Manager::pollerTimerExpired() ERROR: Timer not defined");
        return;
    }

    for (auto& obj : statusObjects)
    {
        // Read sysfs to force kernel to poll OCC
        obj->readOccState();

#ifdef READ_OCC_SENSORS
        // Read occ sensor values
        auto id = obj->getOccInstanceID();
        if (!obj->occActive())
        {
            // Occ not activated
            setSensorValueToNaN(id);
            continue;
        }
        getSensorValues(id, obj->isMasterOcc());
#endif
    }

    // Restart OCC poll timer
    _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
}

#ifdef READ_OCC_SENSORS
void Manager::readTempSensors(const fs::path& path, uint32_t id)
{
    std::regex expr{"temp\\d+_label$"}; // Example: temp5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }

        uint32_t labelValue;

        try
        {
            labelValue = readFile<uint32_t>(file.path());
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            file.path().string(), e.code().value())
                    .c_str());
            continue;
        }

        const std::string& tempLabel = "label";
        const std::string filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        uint32_t fruTypeValue{0};
        try
        {
            fruTypeValue = readFile<uint32_t>(filePathString + "fru_type");
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + "fru_type", e.code().value())
                    .c_str());
            continue;
        }

        std::string sensorPath =
            OCC_SENSORS_ROOT + std::string("/temperature/");

        if (fruTypeValue == VRMVdd)
        {
            sensorPath.append("vrm_vdd" + std::to_string(id) + "_temp");
        }
        else
        {
            uint16_t type = (labelValue & 0xFF000000) >> 24;
            uint16_t instanceID = labelValue & 0x0000FFFF;

            if (type == OCC_DIMM_TEMP_SENSOR_TYPE)
            {
                if (fruTypeValue == fruTypeNotAvailable)
                {
                    // Not all DIMM related temps are available to read
                    // (no _input file in this case)
                    continue;
                }
                auto iter = dimmTempSensorName.find(fruTypeValue);
                if (iter == dimmTempSensorName.end())
                {
                    log<level::ERR>(fmt::format("readTempSensors: Fru type "
                                                "error! fruTypeValue = {}) ",
                                                fruTypeValue)
                                        .c_str());
                    continue;
                }

                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  iter->second);
            }
            else if (type == OCC_CPU_TEMP_SENSOR_TYPE)
            {
                if (fruTypeValue != processorCore)
                {
                    // TODO: support IO ring temp
                    continue;
                }

                sensorPath.append("proc" + std::to_string(id) + "_core" +
                                  std::to_string(instanceID) + "_temp");
            }
            else
            {
                continue;
            }
        }

        uint32_t faultValue{0};
        try
        {
            faultValue = readFile<uint32_t>(filePathString + "fault");
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + "fault", e.code().value())
                    .c_str());
            continue;
        }

        if (faultValue != 0)
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, std::numeric_limits<double>::quiet_NaN());

            open_power::occ::dbus::OccDBusSensors::getOccDBus()
                .setOperationalStatus(sensorPath, false);

            continue;
        }

        double tempValue{0};

        try
        {
            tempValue = readFile<double>(filePathString + "input");
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + "input", e.code().value())
                    .c_str());
            continue;
        }

        open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
            sensorPath, tempValue * std::pow(10, -3));

        open_power::occ::dbus::OccDBusSensors::getOccDBus()
            .setOperationalStatus(sensorPath, true);

        existingSensors[sensorPath] = id;
    }
    return;
}

std::optional<std::string>
    Manager::getPowerLabelFunctionID(const std::string& value)
{
    // If the value is "system", then the FunctionID is "system".
    if (value == "system")
    {
        return value;
    }

    // If the value is not "system", then the label value have 3 numbers, of
    // which we only care about the middle one:
    // <sensor id>_<function id>_<apss channel>
    // eg: The value is "0_10_5" , then the FunctionID is "10".
    if (value.find("_") == std::string::npos)
    {
        return std::nullopt;
    }

    auto powerLabelValue = value.substr((value.find("_") + 1));

    if (powerLabelValue.find("_") == std::string::npos)
    {
        return std::nullopt;
    }

    return powerLabelValue.substr(0, powerLabelValue.find("_"));
}

void Manager::readPowerSensors(const fs::path& path, uint32_t id)
{
    std::regex expr{"power\\d+_label$"}; // Example: power5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }

        std::string labelValue;
        try
        {
            labelValue = readFile<std::string>(file.path());
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readPowerSensors: Failed reading {}, errno = {}",
                            file.path().string(), e.code().value())
                    .c_str());
            continue;
        }

        auto functionID = getPowerLabelFunctionID(labelValue);
        if (functionID == std::nullopt)
        {
            continue;
        }

        const std::string& tempLabel = "label";
        const std::string filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        std::string sensorPath = OCC_SENSORS_ROOT + std::string("/power/");

        auto iter = powerSensorName.find(*functionID);
        if (iter == powerSensorName.end())
        {
            continue;
        }
        sensorPath.append(iter->second);

        double tempValue{0};

        try
        {
            tempValue = readFile<double>(filePathString + "input");
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + "input", e.code().value())
                    .c_str());
            continue;
        }

        open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
            sensorPath, tempValue * std::pow(10, -3) * std::pow(10, -3));

        open_power::occ::dbus::OccDBusSensors::getOccDBus()
            .setOperationalStatus(sensorPath, true);

        existingSensors[sensorPath] = id;
    }
    return;
}

void Manager::setSensorValueToNaN(uint32_t id)
{
    for (const auto& [sensorPath, occId] : existingSensors)
    {
        if (occId == id)
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, std::numeric_limits<double>::quiet_NaN());
        }
    }
    return;
}

void Manager::getSensorValues(uint32_t id, bool masterOcc)
{
    const auto occ = std::string("occ-hwmon.") + std::to_string(id + 1);

    fs::path fileName{OCC_HWMON_PATH + occ + "/hwmon/"};

    // Need to get the hwmonXX directory name, there better only be 1 dir
    assert(std::distance(fs::directory_iterator(fileName),
                         fs::directory_iterator{}) == 1);
    // Now set our path to this full path, including this hwmonXX directory
    fileName = fs::path(*fs::directory_iterator(fileName));

    // Read temperature sensors
    readTempSensors(fileName, id);

    if (masterOcc)
    {
        // Read power sensors
        readPowerSensors(fileName, id);
    }

    return;
}
#endif

} // namespace occ
} // namespace open_power
