#include "config.h"

#include "occ_manager.hpp"

#include "i2c_occ.hpp"
#include "occ_dbus.hpp"
#include "utils.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
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

void Manager::findAndCreateObjects()
{
#ifndef POWER10
    for (auto id = 0; id < MAX_CPUS; ++id)
    {
        // Create one occ per cpu
        auto occ = std::string(OCC_NAME) + std::to_string(id);
        createObjects(occ);
    }
#else
    if (!pmode)
    {
        // Create the power mode object
        pmode = std::make_unique<powermode::PowerMode>(
            *this, powermode::PMODE_PATH, powermode::PIPS_PATH, event);
    }

    if (!fs::exists(HOST_ON_FILE))
    {
        static bool statusObjCreated = false;
        if (!statusObjCreated)
        {
            // Create the OCCs based on on the /dev/occX devices
            auto occs = findOCCsInDev();

            if (occs.empty() || (prevOCCSearch.size() != occs.size()))
            {
                // Something changed or no OCCs yet, try again in 10s.
                // Note on the first pass prevOCCSearch will be empty,
                // so there will be at least one delay to give things
                // a chance to settle.
                prevOCCSearch = occs;

                log<level::INFO>(
                    fmt::format(
                        "Manager::findAndCreateObjects(): Waiting for OCCs (currently {})",
                        occs.size())
                        .c_str());

                discoverTimer->restartOnce(10s);
            }
            else
            {
                // All OCCs appear to be available, create status objects

                // createObjects requires OCC0 first.
                std::sort(occs.begin(), occs.end());

                log<level::INFO>(
                    fmt::format(
                        "Manager::findAndCreateObjects(): Creating {} OCC Status Objects",
                        occs.size())
                        .c_str());
                for (auto id : occs)
                {
                    createObjects(std::string(OCC_NAME) + std::to_string(id));
                }
                statusObjCreated = true;
            }
        }

        if (statusObjCreated)
        {
            static bool tracedHostWait = false;
            if (utils::isHostRunning())
            {
                if (tracedHostWait)
                {
                    log<level::INFO>(
                        "Manager::findAndCreateObjects(): Host is running");
                    tracedHostWait = false;
                }
                waitingForAllOccActiveSensors = true;
                checkAllActiveSensors();
            }
            else
            {
                if (!tracedHostWait)
                {
                    log<level::INFO>(
                        "Manager::findAndCreateObjects(): Waiting for host to start");
                    tracedHostWait = true;
                }
                discoverTimer->restartOnce(30s);
            }
        }
    }
    else
    {
        log<level::INFO>(
            fmt::format(
                "Manager::findAndCreateObjects(): Waiting for {} to complete...",
                HOST_ON_FILE)
                .c_str());
        discoverTimer->restartOnce(10s);
    }
#endif
}

#ifdef POWER10
// Check if all occActive sensors are available
void Manager::checkAllActiveSensors()
{
    static bool allActiveSensorAvailable = false;
    static bool tracedSensorWait = false;

    // Start with the assumption that all are available
    allActiveSensorAvailable = true;
    for (auto& obj : statusObjects)
    {
        // If active sensor is already true, then no need to query sensor
        if (!obj->occActive())
        {
            auto instance = obj->getOccInstanceID();
            // Check if sensor was queued while waiting for discovery
            auto match = queuedActiveState.find(instance);
            if (match != queuedActiveState.end())
            {
                log<level::INFO>(
                    fmt::format(
                        "checkAllActiveSensors(): OCC{} is ACTIVE (queued)",
                        instance)
                        .c_str());
                obj->occActive(true);
            }
            else
            {
                allActiveSensorAvailable = false;
                if (!tracedSensorWait)
                {
                    log<level::INFO>(
                        fmt::format(
                            "checkAllActiveSensors(): Waiting on OCC{} Active sensor",
                            instance)
                            .c_str());
                    tracedSensorWait = true;
                }
                pldmHandle->checkActiveSensor(obj->getOccInstanceID());
                break;
            }
        }
    }

    if (allActiveSensorAvailable)
    {
        // All sensors were found, disable the discovery timer
        discoverTimer.reset();
        waitingForAllOccActiveSensors = false;
        queuedActiveState.clear();

        log<level::INFO>(
            "checkAllActiveSensors(): OCC Active sensors are available");
        tracedSensorWait = false;
    }
    else
    {
        // Not all sensors were available, so keep waiting
        if (!tracedSensorWait)
        {
            log<level::INFO>(
                "checkAllActiveSensors(): Waiting for OCC Active sensors to become available");
            tracedSensorWait = true;
        }
        discoverTimer->restartOnce(10s);
    }
}
#endif

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

int Manager::cpuCreated(sdbusplus::message::message& msg)
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
#ifdef POWER10
        pmode,
#endif
        std::bind(std::mem_fn(&Manager::statusCallBack), this,
                  std::placeholders::_1, std::placeholders::_2)
#ifdef PLDM
            ,
        std::bind(std::mem_fn(&pldm::Interface::resetOCC), pldmHandle.get(),
                  std::placeholders::_1)
#endif
            ));

    // Create the power cap monitor object
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
            *statusObjects.back());
    }

    if (statusObjects.back()->isMasterOcc())
    {
        log<level::INFO>(
            fmt::format("Manager::createObjects(): OCC{} is the master",
                        statusObjects.back()->getOccInstanceID())
                .c_str());
        _pollTimer->setEnabled(false);

#ifdef POWER10
        // Set the master OCC on the PowerMode object
        pmode->setMasterOcc(path);
#endif
    }

    passThroughObjects.emplace_back(std::make_unique<PassThrough>(path.c_str()
#ifdef POWER10
                                                                      ,
                                                                  pmode
#endif
                                                                  ));
}

void Manager::statusCallBack(instanceID instance, bool status)
{
    using InternalFailure =
        sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

    // At this time, it won't happen but keeping it
    // here just in case something changes in the future
    if ((activeCount == 0) && (!status))
    {
        log<level::ERR>(
            fmt::format("Invalid update on OCCActive with OCC{}", instance)
                .c_str());

        elog<InternalFailure>();
    }

    if (status == true)
    {
        // OCC went active
        ++activeCount;

#ifdef POWER10
        if (activeCount == 1)
        {
            // First OCC went active (allow some time for all OCCs to go active)
            waitForAllOccsTimer->restartOnce(60s);
        }
#endif

        if (activeCount == statusObjects.size())
        {
#ifdef POWER10
            // All OCCs are now running
            if (waitForAllOccsTimer->isEnabled())
            {
                // stop occ wait timer
                waitForAllOccsTimer->setEnabled(false);
            }
#endif

            // Verify master OCC and start presence monitor
            validateOccMaster();
        }

        // Start poll timer if not already started
        if (!_pollTimer->isEnabled())
        {
            log<level::INFO>(
                fmt::format("Manager: OCCs will be polled every {} seconds",
                            pollInterval)
                    .c_str());

            // Send poll and start OCC poll timer
            pollerTimerExpired();
        }
    }
    else
    {
        // OCC went away
        --activeCount;

        if (activeCount == 0)
        {
            // No OCCs are running

            // Stop OCC poll timer
            if (_pollTimer->isEnabled())
            {
                log<level::INFO>(
                    "Manager::statusCallBack(): OCCs are not running, stopping poll timer");
                _pollTimer->setEnabled(false);
            }

#ifdef POWER10
            // stop wait timer
            if (waitForAllOccsTimer->isEnabled())
            {
                waitForAllOccsTimer->setEnabled(false);
            }
#endif
        }
#ifdef READ_OCC_SENSORS
        // Clear OCC sensors
        setSensorValueToNonFunctional(instance);
#endif
    }

#ifdef POWER10
    if (waitingForAllOccActiveSensors)
    {
        checkAllActiveSensors();
    }
#endif
}

#ifdef I2C_OCC
void Manager::initStatusObjects()
{
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
#ifdef POWER10
    pmode = std::make_unique<powermode::PowerMode>(*this, powermode::PMODE_PATH,
                                                   powermode::PIPS_PATH);
    // Set the master OCC on the PowerMode object
    pmode->setMasterOcc(path);
#endif
}
#endif

#ifdef PLDM
void Manager::sbeTimeout(unsigned int instance)
{
    auto obj = std::find_if(statusObjects.begin(), statusObjects.end(),
                            [instance](const auto& obj) {
                                return instance == obj->getOccInstanceID();
                            });

    if (obj != statusObjects.end() && (*obj)->occActive())
    {
        log<level::INFO>(
            fmt::format("SBE timeout, requesting HRESET (OCC{})", instance)
                .c_str());

        setSBEState(instance, SBE_STATE_NOT_USABLE);

        pldmHandle->sendHRESET(instance);
    }
}

bool Manager::updateOCCActive(instanceID instance, bool status)
{
    auto obj = std::find_if(statusObjects.begin(), statusObjects.end(),
                            [instance](const auto& obj) {
                                return instance == obj->getOccInstanceID();
                            });

    if (obj != statusObjects.end())
    {
        return (*obj)->occActive(status);
    }
    else
    {
        log<level::WARNING>(
            fmt::format(
                "Manager::updateOCCActive: No status object to update for OCC{} (active={})",
                instance, status)
                .c_str());
        if (status == true)
        {
            // OCC went active
            queuedActiveState.insert(instance);
        }
        else
        {
            auto match = queuedActiveState.find(instance);
            if (match != queuedActiveState.end())
            {
                // OCC was disabled
                queuedActiveState.erase(match);
            }
        }
        return false;
    }
}

void Manager::sbeHRESETResult(instanceID instance, bool success)
{
    if (success)
    {
        log<level::INFO>(
            fmt::format("HRESET succeeded (OCC{})", instance).c_str());

        setSBEState(instance, SBE_STATE_BOOTED);

        return;
    }

    setSBEState(instance, SBE_STATE_FAILED);

    if (sbeCanDump(instance))
    {
        log<level::INFO>(
            fmt::format("HRESET failed (OCC{}), triggering SBE dump", instance)
                .c_str());

        auto& bus = utils::getBus();
        uint32_t src6 = instance << 16;
        uint32_t logId =
            FFDC::createPEL("org.open_power.Processor.Error.SbeChipOpTimeout",
                            src6, "SBE command timeout");

        try
        {
            constexpr auto path = "/org/openpower/dump";
            constexpr auto interface = "xyz.openbmc_project.Dump.Create";
            constexpr auto function = "CreateDump";

            std::string service = utils::getService(path, interface);
            auto method =
                bus.new_method_call(service.c_str(), path, interface, function);

            std::map<std::string, std::variant<std::string, uint64_t>>
                createParams{
                    {"com.ibm.Dump.Create.CreateParameters.ErrorLogId",
                     uint64_t(logId)},
                    {"com.ibm.Dump.Create.CreateParameters.DumpType",
                     "com.ibm.Dump.Create.DumpType.SBE"},
                    {"com.ibm.Dump.Create.CreateParameters.FailingUnitId",
                     uint64_t(instance)},
                };

            method.append(createParams);

            auto response = bus.call(method);
        }
        catch (const sdbusplus::exception::exception& e)
        {
            constexpr auto ERROR_DUMP_DISABLED =
                "xyz.openbmc_project.Dump.Create.Error.Disabled";
            if (e.name() == ERROR_DUMP_DISABLED)
            {
                log<level::INFO>("Dump is disabled, skipping");
            }
            else
            {
                log<level::ERR>("Dump failed");
            }
        }
    }
}

bool Manager::sbeCanDump(unsigned int instance)
{
    struct pdbg_target* proc = getPdbgTarget(instance);

    if (!proc)
    {
        // allow the dump in the error case
        return true;
    }

    try
    {
        if (!openpower::phal::sbe::isDumpAllowed(proc))
        {
            return false;
        }

        if (openpower::phal::pdbg::isSbeVitalAttnActive(proc))
        {
            return false;
        }
    }
    catch (openpower::phal::exception::SbeError& e)
    {
        log<level::INFO>("Failed to query SBE state");
    }

    // allow the dump in the error case
    return true;
}

void Manager::setSBEState(unsigned int instance, enum sbe_state state)
{
    struct pdbg_target* proc = getPdbgTarget(instance);

    if (!proc)
    {
        return;
    }

    try
    {
        openpower::phal::sbe::setState(proc, state);
    }
    catch (const openpower::phal::exception::SbeError& e)
    {
        log<level::ERR>("Failed to set SBE state");
    }
}

struct pdbg_target* Manager::getPdbgTarget(unsigned int instance)
{
    if (!pdbgInitialized)
    {
        try
        {
            openpower::phal::pdbg::init();
            pdbgInitialized = true;
        }
        catch (const openpower::phal::exception::PdbgError& e)
        {
            log<level::ERR>("pdbg initialization failed");
            return nullptr;
        }
    }

    struct pdbg_target* proc = nullptr;
    pdbg_for_each_class_target("proc", proc)
    {
        if (pdbg_target_index(proc) == instance)
        {
            return proc;
        }
    }

    log<level::ERR>("Failed to get pdbg target");
    return nullptr;
}
#endif

void Manager::pollerTimerExpired()
{
    if (!_pollTimer)
    {
        log<level::ERR>(
            "Manager::pollerTimerExpired() ERROR: Timer not defined");
        return;
    }

    for (auto& obj : statusObjects)
    {
        if (!obj->occActive())
        {
            // OCC is not running yet
#ifdef READ_OCC_SENSORS
            auto id = obj->getOccInstanceID();
            setSensorValueToNonFunctional(id);
#endif
            continue;
        }

        // Read sysfs to force kernel to poll OCC
        obj->readOccState();

#ifdef READ_OCC_SENSORS
        // Read occ sensor values
        getSensorValues(obj);
#endif
    }

    if (activeCount > 0)
    {
        // Restart OCC poll timer
        _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
    }
    else
    {
        // No OCCs running, so poll timer will not be restarted
        log<level::INFO>(
            fmt::format(
                "Manager::pollerTimerExpired: poll timer will not be restarted")
                .c_str());
    }
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

        uint32_t labelValue{0};

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
            fruTypeValue = readFile<uint32_t>(filePathString + fruTypeSuffix);
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + fruTypeSuffix, e.code().value())
                    .c_str());
            continue;
        }

        std::string sensorPath =
            OCC_SENSORS_ROOT + std::string("/temperature/");

        std::string dvfsTempPath;

        if (fruTypeValue == VRMVdd)
        {
            sensorPath.append("vrm_vdd" + std::to_string(id) + "_temp");
        }
        else if (fruTypeValue == processorIoRing)
        {
            sensorPath.append("proc" + std::to_string(id) + "_ioring_temp");
            dvfsTempPath = std::string{OCC_SENSORS_ROOT} + "/temperature/proc" +
                           std::to_string(id) + "_ioring_dvfs_temp";
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
                    log<level::ERR>(
                        fmt::format(
                            "readTempSensors: Fru type error! fruTypeValue = {}) ",
                            fruTypeValue)
                            .c_str());
                    continue;
                }

                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  iter->second);
            }
            else if (type == OCC_CPU_TEMP_SENSOR_TYPE)
            {
                if (fruTypeValue == processorCore)
                {
                    // The OCC reports small core temps, of which there are
                    // two per big core.  All current P10 systems are in big
                    // core mode, so use a big core name.
                    uint16_t coreNum = instanceID / 2;
                    uint16_t tempNum = instanceID % 2;
                    sensorPath.append("proc" + std::to_string(id) + "_core" +
                                      std::to_string(coreNum) + "_" +
                                      std::to_string(tempNum) + "_temp");

                    dvfsTempPath = std::string{OCC_SENSORS_ROOT} +
                                   "/temperature/proc" + std::to_string(id) +
                                   "_core_dvfs_temp";
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }
        }

        // The dvfs temp file only needs to be read once per chip per type.
        if (!dvfsTempPath.empty() &&
            !dbus::OccDBusSensors::getOccDBus().hasDvfsTemp(dvfsTempPath))
        {
            try
            {
                auto dvfsValue = readFile<double>(filePathString + maxSuffix);

                dbus::OccDBusSensors::getOccDBus().setDvfsTemp(
                    dvfsTempPath, dvfsValue * std::pow(10, -3));
            }
            catch (const std::system_error& e)
            {
                log<level::DEBUG>(
                    fmt::format(
                        "readTempSensors: Failed reading {}, errno = {}",
                        filePathString + maxSuffix, e.code().value())
                        .c_str());
            }
        }

        uint32_t faultValue{0};
        try
        {
            faultValue = readFile<uint32_t>(filePathString + faultSuffix);
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + faultSuffix, e.code().value())
                    .c_str());
            continue;
        }

        if (faultValue != 0)
        {
            dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, std::numeric_limits<double>::quiet_NaN());

            dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath,
                                                                    false);

            continue;
        }

        double tempValue{0};

        try
        {
            tempValue = readFile<double>(filePathString + inputSuffix);
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readTempSensors: Failed reading {}, errno = {}",
                            filePathString + inputSuffix, e.code().value())
                    .c_str());
            continue;
        }

        dbus::OccDBusSensors::getOccDBus().setValue(
            sensorPath, tempValue * std::pow(10, -3));

        dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath,
                                                                true);

        // At this point, the sensor will be created for sure.
        if (existingSensors.find(sensorPath) == existingSensors.end())
        {
            dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                sensorPath);
        }

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
            tempValue = readFile<double>(filePathString + inputSuffix);
        }
        catch (const std::system_error& e)
        {
            log<level::DEBUG>(
                fmt::format("readPowerSensors: Failed reading {}, errno = {}",
                            filePathString + inputSuffix, e.code().value())
                    .c_str());
            continue;
        }

        dbus::OccDBusSensors::getOccDBus().setUnit(
            sensorPath, "xyz.openbmc_project.Sensor.Value.Unit.Watts");

        dbus::OccDBusSensors::getOccDBus().setValue(
            sensorPath, tempValue * std::pow(10, -3) * std::pow(10, -3));

        dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath,
                                                                true);

        if (existingSensors.find(sensorPath) == existingSensors.end())
        {
            dbus::OccDBusSensors::getOccDBus().setChassisAssociation(
                sensorPath);
        }

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
            dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, std::numeric_limits<double>::quiet_NaN());
        }
    }
    return;
}

void Manager::setSensorValueToNonFunctional(uint32_t id) const
{
    for (const auto& [sensorPath, occId] : existingSensors)
    {
        if (occId == id)
        {
            dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, std::numeric_limits<double>::quiet_NaN());

            dbus::OccDBusSensors::getOccDBus().setOperationalStatus(sensorPath,
                                                                    false);
        }
    }
    return;
}

void Manager::getSensorValues(std::unique_ptr<Status>& occ)
{
    static bool tracedError[8] = {0};
    const fs::path sensorPath = occ->getHwmonPath();
    const uint32_t id = occ->getOccInstanceID();

    if (fs::exists(sensorPath))
    {
        // Read temperature sensors
        readTempSensors(sensorPath, id);

        if (occ->isMasterOcc())
        {
            // Read power sensors
            readPowerSensors(sensorPath, id);
        }
        tracedError[id] = false;
    }
    else
    {
        if (!tracedError[id])
        {
            log<level::ERR>(
                fmt::format(
                    "Manager::getSensorValues: OCC{} sensor path missing: {}",
                    id, sensorPath.c_str())
                    .c_str());
            tracedError[id] = true;
        }
    }

    return;
}
#endif

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
            log<level::DEBUG>(fmt::format("readAltitude: sensor={} ({}m)",
                                          sensorVal, altitude)
                                  .c_str());
            traceAltitudeErr = true;
        }
        else
        {
            if (traceAltitudeErr)
            {
                traceAltitudeErr = false;
                log<level::DEBUG>(
                    fmt::format("Invalid altitude value: {}", sensorVal)
                        .c_str());
            }
        }
    }
    catch (const sdbusplus::exception::exception& e)
    {
        if (traceAltitudeErr)
        {
            traceAltitudeErr = false;
            log<level::INFO>(
                fmt::format("Unable to read Altitude: {}", e.what()).c_str());
        }
        altitude = 0xFFFF; // not available
    }
}

// Callback function when ambient temperature changes
void Manager::ambientCallback(sdbusplus::message::message& msg)
{
    double currentTemp = 0;
    uint8_t truncatedTemp = 0xFF;
    std::string msgSensor;
    std::map<std::string, std::variant<double>> msgData;
    msg.read(msgSensor, msgData);

    auto valPropMap = msgData.find(AMBIENT_PROP);
    if (valPropMap == msgData.end())
    {
        log<level::DEBUG>("ambientCallback: Unknown ambient property changed");
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
        log<level::DEBUG>(
            fmt::format("ambientCallback: Ambient change from {} to {}C",
                        ambient, currentTemp)
                .c_str());

        ambient = truncatedTemp;
        if (altitude == 0xFFFF)
        {
            // No altitude yet, try reading again
            readAltitude();
        }

        log<level::DEBUG>(
            fmt::format("ambientCallback: Ambient: {}C, altitude: {}m", ambient,
                        altitude)
                .c_str());
#ifdef POWER10
        // Send ambient and altitude to all OCCs
        for (auto& obj : statusObjects)
        {
            if (obj->occActive())
            {
                obj->sendAmbient(ambient, altitude);
            }
        }
#endif // POWER10
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

#ifdef POWER10
void Manager::occsNotAllRunning()
{
    // Function will also gets called when occ-control app gets
    // restarted. (occ active sensors do not change, so the Status
    // object does not call Manager back for all OCCs)

    if (activeCount != statusObjects.size())
    {
        // Not all OCCs went active
        log<level::WARNING>(
            fmt::format(
                "occsNotAllRunning: Active OCC count ({}) does not match expected count ({})",
                activeCount, statusObjects.size())
                .c_str());
        // Procs may be garded, so may not need reset.
    }

    validateOccMaster();
}
#endif // POWER10

// Verify single master OCC and start presence monitor
void Manager::validateOccMaster()
{
    int masterInstance = -1;
    for (auto& obj : statusObjects)
    {
        auto instance = obj->getOccInstanceID();
#ifdef POWER10
        if (!obj->occActive())
        {
            if (utils::isHostRunning())
            {
                // Check if sensor was queued while waiting for discovery
                auto match = queuedActiveState.find(instance);
                if (match != queuedActiveState.end())
                {
                    log<level::INFO>(
                        fmt::format(
                            "validateOccMaster: OCC{} is ACTIVE (queued)",
                            instance)
                            .c_str());
                    obj->occActive(true);
                }
                else
                {
                    // OCC does not appear to be active yet, check active sensor
                    pldmHandle->checkActiveSensor(instance);
                    if (obj->occActive())
                    {
                        log<level::INFO>(
                            fmt::format(
                                "validateOccMaster: OCC{} is ACTIVE after reading sensor",
                                instance)
                                .c_str());
                    }
                }
            }
            else
            {
                log<level::WARNING>(
                    fmt::format(
                        "validateOccMaster: HOST is not running (OCC{})",
                        instance)
                        .c_str());
                return;
            }
        }
#endif // POWER10

        if (obj->isMasterOcc())
        {
            obj->addPresenceWatchMaster();

            if (masterInstance == -1)
            {
                masterInstance = instance;
            }
            else
            {
                log<level::ERR>(
                    fmt::format(
                        "validateOccMaster: Multiple OCC masters! ({} and {})",
                        masterInstance, instance)
                        .c_str());
                // request reset
                obj->deviceError();
            }
        }
    }

    if (masterInstance < 0)
    {
        log<level::ERR>(
            fmt::format("validateOccMaster: Master OCC not found! (of {} OCCs)",
                        statusObjects.size())
                .c_str());
        // request reset
        statusObjects.front()->deviceError();
    }
    else
    {
        log<level::INFO>(
            fmt::format("validateOccMaster: OCC{} is master of {} OCCs",
                        masterInstance, activeCount)
                .c_str());
    }
}

void Manager::updatePcapBounds() const
{
    if (pcap)
    {
        pcap->updatePcapBounds();
    }
}

} // namespace occ
} // namespace open_power
