#include "occ_device.hpp"

#include "occ_manager.hpp"
#include "occ_status.hpp"

#include <fmt/core.h>

#include <phosphor-logging/log.hpp>

#include <iostream>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

void Device::addErrorWatch(bool poll)
{
    unsigned int retriesRemaining = 15;
    do
    {
        try
        {
            throttleProcTemp.addWatch(poll);
            break;
        }
        catch (const OpenFailure& e)
        {
            log<level::ERR>(
                fmt::format(
                    "addErrorWatch() OpenFailure - {}, using oc_dvfs_ot ({} retries)",
                    e.what(), --retriesRemaining)
                    .c_str());
            // try the old kernel version
            throttleProcTemp.setFile(devPath / "occ_dvfs_ot");
            throttleProcTemp.addWatch(poll);
        }
        catch (const ReadFailure& e)
        {
            // keep trying
            log<level::ERR>(
                fmt::format("addErrorWatch() ReadFailure - {} ({} retries)",
                            e.what(), --retriesRemaining)
                    .c_str());
        }
        sleep(2);
    } while (retriesRemaining);

    if (retriesRemaining == 0)
    {
        log<level::ERR>("addErrorWatch() FAILED to add watch!");
        // TODO: force assert???
    }

    throttleProcPower.addWatch(poll);
    throttleMemTemp.addWatch(poll);

    try
    {
        ffdc.addWatch(poll);
    }
    catch (const OpenFailure& e)
    {
        // nothing to do if there is no FFDC file
    }

    try
    {
        timeout.addWatch(poll);
    }
    catch (const std::exception& e)
    {
        // nothing to do if there is no SBE timeout file
    }

    error.addWatch(poll);
}

std::string Device::getPathBack(const fs::path& path)
{
    if (path.empty())
    {
        return std::string();
    }

    // Points to the last element in the path
    auto conf = --path.end();

    if (conf->empty() && conf != path.begin())
    {
        return *(--conf);
    }
    else
    {
        return *conf;
    }
}

bool Device::master() const
{
    int master;
    auto masterFile = devPath / "occ_master";
    std::ifstream file(masterFile, std::ios::in);

    if (!file)
    {
        return false;
    }

    file >> master;
    file.close();
    return (master != 0);
}

void Device::errorCallback(bool error)
{
    if (error)
    {
        statusObject.deviceError();
    }
}

#ifdef PLDM
void Device::timeoutCallback(bool error)
{
    if (error)
    {
        managerObject.sbeTimeout(instance);
    }
}
#endif

void Device::throttleProcTempCallback(bool error)
{
    statusObject.throttleProcTemp(error);
}

void Device::throttleProcPowerCallback(bool error)
{
    statusObject.throttleProcPower(error);
}

void Device::throttleMemTempCallback(bool error)
{
    statusObject.throttleMemTemp(error);
}

} // namespace occ
} // namespace open_power
