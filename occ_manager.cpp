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

/** @brief Default processor fru type */
constexpr unsigned int procFruType = 0;

/** @brief Default dimm fru type */
constexpr unsigned int dimmFruType = 2;

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
        getProcDimmTemp();
    }

    // Restart OCC poll timer
    _pollTimer->restartOnce(std::chrono::seconds(pollInterval));
}

void Manager::readProcDimmTemp(const fs::path& path, const unsigned int& id)
{
    std::string state;
    const int open_errno = errno;
    std::regex expr{"temp\\d+_label$"};
    for (auto& file : fs::directory_iterator(path))
    {
        if (std::regex_search(file.path().string(), expr))
        {
            std::ifstream fileOpen(file.path(), std::ios::in);

            if (fileOpen)
            {
                fileOpen >> state;

                fileOpen.close();
            }
            else
            {
                // If not able to read, OCC may be offline
                log<level::DEBUG>(
                    fmt::format("readProcDimmTemp: open failed(errno = {}) ",
                                open_errno)
                        .c_str());
            }

            auto labelValue = open_power::occ::utils::checkLabelValue(state);
            auto& [type, instanceID] = *labelValue;

            std::string sensorPath;

            const std::string& tempLabel = "label";

            std::string tmp = file.path().string();
            std::string fruTypePathTmp =
                tmp.substr(0, tmp.length() - tempLabel.length());
            std::string fruTypePath = fruTypePathTmp + "fru_type";
            std::ifstream fruTypeFile(fruTypePath, std::ios::in);
            unsigned int fruTypeValue;
            if (fruTypeFile)
            {
                fruTypeFile >> fruTypeValue;

                fruTypeFile.close();
            }
            else
            {
                // If not able to read, OCC may be offline
                log<level::DEBUG>(
                    fmt::format("readProcDimmTemp: open failed(errno = {}) ",
                                open_errno)
                        .c_str());
                continue;
            }

            if (type == OCC_DIMM_TEMP_SENSOR_TYPE)
            {
                if (fruTypeValue != dimmFruType)
                {
                    continue;
                }

                sensorPath = "/xyz/openbmc_project/sensors/"
                             "temperature/dimm" +
                             std::to_string(instanceID) + "_dram_temp";
            }
            else if (type == OCC_CPU_TEMP_SENSOR_TYPE)
            {
                if (fruTypeValue != procFruType)
                {
                    continue;
                }
                sensorPath = "/xyz/openbmc_project/sensors/"
                             "temperature/proc" +
                             std::to_string(id) + "_core" +
                             std::to_string(instanceID) + "_temp";
            }
            else
            {
                continue;
            }

            std::string tmpFaultPath = file.path().string();
            std::string faultPathTmp = tmpFaultPath.substr(
                0, tmpFaultPath.length() - tempLabel.length());
            std::string faultPath = fruTypePathTmp + "fault";

            std::ifstream faultPathFile(faultPath, std::ios::in);
            unsigned int faultValue;
            if (faultPathFile)
            {
                faultPathFile >> faultValue;

                faultPathFile.close();
            }

            if (faultValue != 0)
            {
                // Todo: if faultValue != 0, set value to NaN.
                const double value = 0;

                open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                    sensorPath, value);

                continue;
            }

            std::string tmpInputPath = file.path().string();
            std::string inputPathTmp = tmpInputPath.substr(
                0, tmpInputPath.length() - tempLabel.length());
            std::string inputFilePath = inputPathTmp + "input";

            std::ifstream inputFile(inputFilePath, std::ios::in);
            double tempValue;
            if (inputFile)
            {
                inputFile >> tempValue;

                open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                    sensorPath, tempValue * std::pow(10, -3));

                existingSensors[sensorPath] = id;

                inputFile.close();
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
    }
    return;
}

void Manager::setTempToNaN(const unsigned int& id)
{
    const double value = 0; // Todo: need set value to NaN.
    std::map<std::string, unsigned int>::iterator iter;
    for (iter = existingSensors.begin(); iter != existingSensors.end(); iter++)
    {
        if (iter->second == id)
        {
            open_power::occ::dbus::OccDBusSensors::getOccDBus().setValue(
                iter->first, value);
        }
    }
    return;
}

void Manager::getProcDimmTemp()
{
    const int open_errno = errno;
    for (auto id = 0; id < MAX_CPUS; ++id)
    {
        const auto occ = std::string("occ-hwmon.") + std::to_string(id + 1);
        const std::string occActivePath = OCC_HWMON_PATH + occ + "/occ_active";
        std::ifstream occActiveFile(occActivePath, std::ios::in);
        unsigned int occActiveValue;
        if (occActiveFile)
        {
            occActiveFile >> occActiveValue;

            occActiveFile.close();

            if (occActiveValue != 1)
            {
                // Occ not activated
                setTempToNaN(id);

                continue;
            }
        }
        else
        {
            // If not able to read, OCC may be offline
            log<level::DEBUG>(
                fmt::format("getProcDimmTemp: open failed(errno = {}) ",
                            open_errno)
                    .c_str());

            // Occ not activated
            setTempToNaN(id);

            continue;
        }

        std::unique_ptr<fs::path> fileName =
            std::make_unique<fs::path>(OCC_HWMON_PATH);

        *fileName /= occ;

        *fileName /= "/hwmon/";

        // Need to get the hwmonXX directory name, there better only be 1 dir
        assert(std::distance(fs::directory_iterator(*fileName),
                             fs::directory_iterator{}) == 1);
        // Now set our path to this full path, including this hwmonXX directory
        fileName =
            std::make_unique<fs::path>(*fs::directory_iterator(*fileName));

        readProcDimmTemp(*fileName, id);
    }

    return;
}

} // namespace occ
} // namespace open_power
