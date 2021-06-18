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

#ifdef READ_OCC_TEMP
enum occFruType
{
    processorCore = 0,
    internalMemCtlr,
    dimm,
    memCtrlAndDimm,
    PMIC = 7,
    memCtlrExSensor = 8

};
#endif

using namespace phosphor::logging;

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

#ifdef READ_OCC_TEMP
        // Read Proc and Dimm temp
        auto id = obj->getOccInstanceID();
        if (!obj->occActive())
        {
            // Occ not activated
            setTempToNaN(id);
            continue;
        }
        getProcDimmTemp(id);
#endif
    }

    // Restart OCC poll timer
    _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
}

#ifdef READ_OCC_TEMP
void Manager::readProcDimmTemp(const fs::path& path, uint32_t id)
{
    std::string state;
    const int open_errno = errno;
    std::regex expr{"temp\\d+_label$"}; // Example: temp5_label
    for (auto& file : fs::directory_iterator(path))
    {
        if (!std::regex_search(file.path().string(), expr))
        {
            continue;
        }
        std::ifstream fileOpen(file.path(), std::ios::in);
        if (!fileOpen)
        {
            // If not able to read, OCC may be offline
            log<level::DEBUG>(
                fmt::format("readProcDimmTemp: open failed(errno = {}) ",
                            open_errno)
                    .c_str());

            continue;
        }
        fileOpen >> state;
        fileOpen.close();

        auto labelValue = open_power::occ::utils::checkLabelValue(state);
        if (labelValue == std::nullopt)
        {
            continue;
        }
        auto& [type, instanceID] = *labelValue;

        const std::string& tempLabel = "label";

        const std::string& filePathString = file.path().string().substr(
            0, file.path().string().length() - tempLabel.length());

        std::ifstream fruTypeFile(filePathString + "fru_type", std::ios::in);
        uint32_t fruTypeValue;
        if (!fruTypeFile)
        {
            // If not able to read, OCC may be offline
            log<level::DEBUG>(
                fmt::format("readProcDimmTemp: open failed(errno = {}) ",
                            open_errno)
                    .c_str());
            continue;
        }
        fruTypeFile >> fruTypeValue;
        fruTypeFile.close();

        std::string sensorPath = OCC_SENSOR_PATH_ROOT;

        if (type == OCC_DIMM_TEMP_SENSOR_TYPE)
        {
            if (fruTypeValue == internalMemCtlr)
            {
                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  "_intmb_temp");
            }
            else if (fruTypeValue == dimm)
            {
                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  "_dram_temp");
            }
            else if (fruTypeValue == memCtrlAndDimm)
            {
                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  "_dram_extmb_temp");
            }
            else if (fruTypeValue == PMIC)
            {
                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  "_pmic_temp");
            }
            else if (fruTypeValue == memCtlrExSensor)
            {
                sensorPath.append("dimm" + std::to_string(instanceID) +
                                  "_extmb_temp");
            }
            else
            {
                log<level::DEBUG>(
                    fmt::format(
                        "readProcDimmTemp: Fru type error! fruTypeValue = {}) ",
                        fruTypeValue)
                        .c_str());
                continue;
            }
        }
        else if (type == OCC_CPU_TEMP_SENSOR_TYPE)
        {
            if (fruTypeValue != processorCore)
            {
                log<level::DEBUG>(
                    fmt::format(
                        "readProcDimmTemp: Fru type error! fruTypeValue = {}) ",
                        fruTypeValue)
                        .c_str());
                continue;
            }

            sensorPath.append("proc" + std::to_string(id) + "_core" +
                              std::to_string(instanceID) + "_temp");
        }
        else
        {
            continue;
        }

        std::ifstream faultPathFile(filePathString + "fault", std::ios::in);
        uint32_t faultValue;
        if (faultPathFile)
        {
            faultPathFile >> faultValue;
            faultPathFile.close();

            if (faultValue != 0)
            {
                open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                    sensorPath, std::numeric_limits<double>::quiet_NaN());

                continue;
            }
        }

        std::ifstream inputFile(filePathString + "input", std::ios::in);
        double tempValue;
        if (inputFile)
        {
            inputFile >> tempValue;

            inputFile.close();

            open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                sensorPath, tempValue * std::pow(10, -3));

            existingSensors[sensorPath] = id;
        }
        else
        {
            // If not able to read, OCC may be offline
            log<level::DEBUG>(
                fmt::format("readProcDimmTemp: open failed(errno = {}) ",
                            open_errno)
                    .c_str());
        }
    }
    return;
}

void Manager::setTempToNaN(uint32_t id)
{
    std::map<std::string, uint32_t>::iterator iter;
    for (iter = existingSensors.begin(); iter != existingSensors.end(); iter++)
    {
        if (iter->second == id)
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                iter->first, std::numeric_limits<double>::quiet_NaN());
        }
    }
    return;
}

void Manager::getProcDimmTemp(uint32_t id)
{
    const auto occ = std::string("occ-hwmon.") + std::to_string(id + 1);

    fs::path fileName{OCC_HWMON_PATH + occ + "/hwmon/"};

    // Need to get the hwmonXX directory name, there better only be 1 dir
    assert(std::distance(fs::directory_iterator(fileName),
                         fs::directory_iterator{}) == 1);
    // Now set our path to this full path, including this hwmonXX directory
    fileName = fs::path(*fs::directory_iterator(fileName));

    readProcDimmTemp(fileName, id);

    return;
}
#endif

} // namespace occ
} // namespace open_power
