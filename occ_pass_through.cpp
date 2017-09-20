#include <memory>
#include <algorithm>
#include <fcntl.h>
#include <errno.h>
#include <string>
#include <unistd.h>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Control/Device/error.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include "occ_pass_through.hpp"
#include "config.h"
namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

PassThrough::PassThrough(
    sdbusplus::bus::bus& bus,
    const char* path) :
    Iface(bus, path),
    path(path),
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    activeStatusSignal(
            bus,
            sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
            std::bind(std::mem_fn(&PassThrough::activeStatusEvent),
                this, std::placeholders::_1))
{
    // Nothing to do.
}

void PassThrough::openDevice()
{
    using InternalFailure = sdbusplus::xyz::openbmc_project::
                                Common::Error::InternalFailure;
    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        // This must be an internal failure since we get
        // here only after making sure it's ready
        log<level::ERR>("Opening passthrough device failed",
                entry("DEVICE=%s",devicePath.c_str()),
                entry("ERRNO=%d",errno));
        elog<InternalFailure>();
    }
    return;
}

void PassThrough::closeDevice()
{
    if (fd >= 0)
    {
        close(fd);
    }
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    // Device failures
    using ReadFailure = sdbusplus::xyz::openbmc_project::
                            Control::Device::Error::ReadFailure;
    using WriteFailure = sdbusplus::xyz::openbmc_project::
                            Control::Device::Error::WriteFailure;

    std::vector<int32_t> response {};

    // OCC only understands [bytes] so need array of bytes. Doing this
    // because rest-server currently treats all int* as 32 bit integer.
    std::vector<uint8_t> cmdInBytes;
    cmdInBytes.resize(command.size());

    // Populate uint8_t version of vector.
    std::transform(command.begin(), command.end(), cmdInBytes.begin(),
            [](decltype(cmdInBytes)::value_type x){return x;});

    ssize_t size = cmdInBytes.size() * sizeof(decltype(cmdInBytes)::value_type);
    auto rc = write(fd, cmdInBytes.data(), size);
    if (rc < 0 || (rc != size))
    {
        log<level::ERR>("Writing to OCC failed");
        elog<WriteFailure>(
            phosphor::logging::xyz::openbmc_project::Common::
                Callout::Device::CALLOUT_ERRNO(errno),
            phosphor::logging::xyz::openbmc_project::Common::
                Callout::Device::CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }

    // Now read the response. This would be the content of occ-sram
    while(1)
    {
        uint8_t data {};
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
            log<level::ERR>("Reading from OCC failed");
            elog<ReadFailure>(
                phosphor::logging::xyz::openbmc_project::Common::
                    Callout::Device::CALLOUT_ERRNO(errno),
                phosphor::logging::xyz::openbmc_project::Common::
                    Callout::Device::CALLOUT_DEVICE_PATH(devicePath.c_str()));
        }
    }

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
            this->openDevice();
        }
        else
        {
            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
