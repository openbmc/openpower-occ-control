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
#include <regex>

namespace open_power
{
namespace occ
{

constexpr uint32_t fruTypeNotAvailable = 0xFF;
constexpr auto fruTypeSuffix = "fru_type";
constexpr auto faultSuffix = "fault";
constexpr auto inputSuffix = "input";

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
    // Create the OCCs based on on the /dev/occX devices
    auto occs = findOCCsInDev();

    if (occs.empty() || (prevOCCSearch.size() != occs.size()))
    {
        // Something changed or no OCCs yet, try again in 10s.
        // Note on the first pass prevOCCSearch will be empty,
        // so there will be at least one delay to give things
        // a chance to settle.
        prevOCCSearch = occs;

        using namespace std::literals::chrono_literals;
        discoverTimer->restartOnce(10s);
    }
    else
    {
        discoverTimer.reset();

        // createObjects requires OCC0 first.
        std::sort(occs.begin(), occs.end());

        for (auto id : occs)
        {
            createObjects(std::string(OCC_NAME) + std::to_string(id));
        }
    }
#endif
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
    // Create the idle power saver monitor object for master occ (0)
    if (!pips)
    {
        pips = std::make_unique<open_power::occ::powermode::PowerIPS>(
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
        log<level::INFO>(
            fmt::format(
                "Manager::statusCallBack(): {} OCCs will be polled every {} seconds",
                activeCount, pollInterval)
                .c_str());

        // Send poll and start OCC poll timer
        pollerTimerExpired();
    }
    else if ((_pollTimer->isEnabled()) && (activeCount == 0))
    {
        // Stop OCC poll timer
        log<level::INFO>(
            "Manager::statusCallBack(): OCCs are not running, stopping poll timer");
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
    pips = std::make_unique<open_power::occ::powermode::PowerIPS>(
        *statusObjects.front());
#endif
}
#endif

#ifdef PLDM
void Manager::sbeTimeout(unsigned int instance)
{
    log<level::INFO>("SBE timeout, requesting HRESET",
                     entry("SBE=%d", instance));

    setSBEState(instance, SBE_STATE_NOT_USABLE);

    pldmHandle->sendHRESET(instance);
}

bool Manager::updateOCCActive(instanceID instance, bool status)
{
    return (statusObjects[instance])->occActive(status);
}

void Manager::sbeHRESETResult(instanceID instance, bool success)
{
    if (success)
    {
        log<level::INFO>("HRESET succeeded", entry("SBE=%d", instance));

        setSBEState(instance, SBE_STATE_BOOTED);

        return;
    }

    setSBEState(instance, SBE_STATE_FAILED);

    if (sbeCanDump(instance))
    {
        constexpr auto path = "/org/openpower/dump";
        constexpr auto interface = "xyz.openbmc_project.Dump.Create";
        constexpr auto function = "CreateDump";

        log<level::INFO>("HRESET failed, triggering SBE dump",
                         entry("SBE=%d", instance));

        auto& bus = utils::getBus();
        uint32_t src6 = instance << 16;
        uint32_t logId =
            FFDC::createPEL("org.open_power.Processor.Error.SbeChipOpTimeout",
                            src6, "SBE command timeout");

        try
        {
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
    if (activeCount == 0)
    {
        // No OCCs running, so poll timer will not be restarted
        log<level::INFO>(
            "Manager::pollerTimerExpire(): No OCCs running, poll timer not restarted");
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
                if (fruTypeValue != processorCore)
                {
                    // TODO: support IO ring temp
                    continue;
                }

                // The OCC reports small core temps, of which there are
                // two per big core.  All current P10 systems are in big
                // core mode, so use a big core name.
                uint16_t coreNum = instanceID / 2;
                uint16_t tempNum = instanceID % 2;
                sensorPath.append("proc" + std::to_string(id) + "_core" +
                                  std::to_string(coreNum) + "_" +
                                  std::to_string(tempNum) + "_temp");
            }
            else
            {
                continue;
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

        // At this point, the sensor will be created for sure.
        if (existingSensors.find(sensorPath) == existingSensors.end())
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus()
                .setChassisAssociation(sensorPath);
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

        open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
            sensorPath, tempValue * std::pow(10, -3) * std::pow(10, -3));

        open_power::occ::dbus::OccDBusSensors::getOccDBus()
            .setOperationalStatus(sensorPath, true);

        if (existingSensors.find(sensorPath) == existingSensors.end())
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus()
                .setChassisAssociation(sensorPath);
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
