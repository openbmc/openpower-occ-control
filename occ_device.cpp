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
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    auto retries = 3;
    auto fd = open(fileName.c_str(), O_WRONLY);
    auto delay = std::chrono::milliseconds{100};

    if (fd < 0)
    {
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::
                OpenFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::
                OpenFailure::CALLOUT_DEVICE_PATH(fileName.c_str()));
        return;
    }

    // OCC / FSI have intermittent issues so retry all binds
    while (true)
    {
        auto rc = ::write(fd, data.data(), data.size());
        if (rc < 0)
        {
            retries--;
            if (retries == 0)
            {
                elog<WriteFailure>(
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_ERRNO(errno),
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_DEVICE_PATH(fileName.c_str()));
                break;
            }
            continue;
            std::this_thread::sleep_for(delay);
        }
        break;
    }

    close(fd);
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
