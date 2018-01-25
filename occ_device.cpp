#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <org/open_power/OCC/Device/error.hpp>
#include "elog-errors.hpp"
#include "occ_device.hpp"
#include "occ_status.hpp"

namespace open_power
{
namespace occ
{

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

void Device::write(const fs::path& fileName, const std::string& data)
{
    using namespace phosphor::logging;

    // If there is an error, move the exception all the way up
    std::ofstream file(fileName, std::ios::out);

    if (file.bad())
    {
        log<level::INFO>("Failed to write; open",
                         entry("PATH=%s", fileName.c_str()));
    }

    file << data;
    if (file.bad())
    {
        log<level::INFO>("Failed to write; write",
                         entry("PATH=%s", fileName.c_str()),
                         entry("DATA=%s", data.c_str()));
    }

    file.close();
}

bool Device::master() const
{
    int master;
    auto masterFile = fs::path(DEV_PATH) / config / "occ_master";
    std::ifstream file(masterFile, std::ios::in);

    if (!file)
    {
        return false;
    }

    file >> master;
    file.close();
    return (master != 0);
}

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
