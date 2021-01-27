#include "config.h"

#include "occ_pass_through.hpp"

#include "elog-errors.hpp"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <string>
#include <iostream>
#include <fmt/core.h>

namespace open_power
{
namespace occ
{

PassThrough::PassThrough(sdbusplus::bus::bus& bus, const char* path) :
    Iface(bus, path), path(path),
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    activeStatusSignal(
        bus, sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&PassThrough::activeStatusEvent), this,
                  std::placeholders::_1))
{
    // Nothing to do.
    using namespace phosphor::logging;
    log<level::INFO>(fmt::format("PassThrough::PassThrough(path={}, devicePath={})",
                                 this->path, devicePath).c_str());
}

void PassThrough::openDevice()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    if (!occActive)
    {
        log<level::INFO>("OCC is inactive; cannot perform pass-through");
        return;
    }

    log<level::INFO>(fmt::format("PassThrough::openDevice: calling open {}", devicePath).c_str());
    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        const int open_errno = errno;
        log<level::ERR>("PassThrough::openDevice: open failed", entry("ERRNO=%d", open_errno));
        // This would log and terminate since its not handled.
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_ERRNO(open_errno),
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    else
        log<level::INFO>("PassThrough::openDevice: open success");

    return;
}

void PassThrough::closeDevice()
{
    using namespace phosphor::logging;
    if (fd >= 0)
    {
        log<level::INFO>("PassThrough::closeDevice: calling close()");
        close(fd);
        fd = -1;
    }
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    std::vector<int32_t> response{};

    log<level::INFO>("PassThrough::send: calling openDevice()");
    openDevice();

    if (fd < 0)
    {
        // OCC is inactive; empty response
        return response;
    }

    // OCC only understands [bytes] so need array of bytes. Doing this
    // because rest-server currently treats all int* as 32 bit integer.
    std::vector<uint8_t> cmdInBytes;
    cmdInBytes.resize(command.size());

    // Populate uint8_t version of vector.
    std::transform(command.begin(), command.end(), cmdInBytes.begin(),
                   [](decltype(cmdInBytes)::value_type x) { return x; });

    ssize_t size = cmdInBytes.size() * sizeof(decltype(cmdInBytes)::value_type);
    log<level::INFO>("PassThrough::send: calling write()");
    auto rc = write(fd, cmdInBytes.data(), size);
    if (rc < 0 || (rc != size))
    {
        const int write_errno = errno;
        log<level::ERR>("PassThrough::send: write failed");
        // This would log and terminate since its not handled.
        elog<WriteFailure>(
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_ERRNO(write_errno),
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    else
    {
        log<level::INFO>("PassThrough::send: write succeeded");
    }

    // Now read the response. This would be the content of occ-sram
    while (1)
    {
        uint8_t data{};
        auto len = read(fd, &data, sizeof(data));
        const int read_errno = errno;
        if (len > 0)
        {
            response.emplace_back(data);
        }
        else if (len < 0 && read_errno == EAGAIN)
        {
            // We may have data coming still.
            // This driver does not need a sleep for a retry.
            continue;
        }
        else if (len == 0)
        {
            log<level::INFO>("PassThrough::send: read completed");
            // We have read all that we can.
            break;
        }
        else
        {
            log<level::ERR>("PassThrough::send: read failed");
            // This would log and terminate since its not handled.
            elog<ReadFailure>(
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_ERRNO(read_errno),
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_DEVICE_PATH(devicePath.c_str()));
        }
    }

    closeDevice();

    return response;
}

// Called at OCC Status change signal
void PassThrough::activeStatusEvent(sdbusplus::message::message& msg)
{
    using namespace phosphor::logging;
    std::string statusInterface;
    std::map<std::string, std::variant<bool>> msgData;
    msg.read(statusInterface, msgData);

    auto propertyMap = msgData.find("OccActive");
    if (propertyMap != msgData.end())
    {
        // Extract the OccActive property
        if (std::get<bool>(propertyMap->second))
        {
            occActive = true;

            log<level::INFO>(fmt::format("PassThrough::activeStatusEvent: {} is ACTIVE", devicePath).c_str());
        }
        else
        {
            occActive = false;

            log<level::INFO>(fmt::format("PassThrough::activeStatusEvent: {} is DISABLED", devicePath).c_str());

            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
