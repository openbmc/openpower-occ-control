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

    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        // This would log and terminate since its not handled.
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    return;
}

void PassThrough::closeDevice()
{
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
    }
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    std::vector<int32_t> response{};

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
    auto rc = write(fd, cmdInBytes.data(), size);
    if (rc < 0 || (rc != size))
    {
        // This would log and terminate since its not handled.
        elog<WriteFailure>(
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }

    // Now read the response. This would be the content of occ-sram
    while (1)
    {
        uint8_t data{};
        auto len = read(fd, &data, sizeof(data));
        if (len > 0)
        {
            response.emplace_back(data);
        }
        else if (len < 0 && errno == EAGAIN)
        {
            // We may have data coming still.
            // This driver does not need a sleep for a retry.
            continue;
        }
        else if (len == 0)
        {
            // We have read all that we can.
            break;
        }
        else
        {
            // This would log and terminate since its not handled.
            elog<ReadFailure>(
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_ERRNO(errno),
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
    std::string statusInterface;
    std::map<std::string, sdbusplus::message::variant<bool>> msgData;
    msg.read(statusInterface, msgData);

    auto propertyMap = msgData.find("OccActive");
    if (propertyMap != msgData.end())
    {
        // Extract the OccActive property
        if (sdbusplus::message::variant_ns::get<bool>(propertyMap->second))
        {
            occActive = true;
        }
        else
        {
            occActive = false;
            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
