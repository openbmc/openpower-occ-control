#include "config.h"

#include "occ_command.hpp"

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
//#include <fmt/core.h>

//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::send: calling openDevice()
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: 3:PassThrough::openDevice: calling open device=/dev/occ1
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::openDevice: calling open /dev/occ1
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::openDevice: open success
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::send: calling write()
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::send: write succeeded
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::send: read completed
//Mar 03 20:09:02 rain147bmc openpower-occ-control[3075]: PassThrough::closeDevice: calling close()

namespace open_power
{
namespace occ
{

OccCommand::OccCommand(sdbusplus::bus::bus& bus, const char* path) :
    path(path),
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    activeStatusSignal(
        bus, sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&OccCommand::activeStatusEvent), this,
                  std::placeholders::_1))
{
    // Nothing to do.
    using namespace phosphor::logging;
    std::string m = "OccCommand::OccCommandr(path=" + this->path + ", devicePath=" + devicePath.c_str() + ")";
    log<level::INFO>(m.c_str());
}

void OccCommand::openDevice()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    if (!occActive)
    {
        log<level::INFO>("OccCommand::openDevice() - OCC is not active; cannot send commands");
        return;
    }

    std::string m = "OccCommand::openDevice: calling open " + devicePath;
    log<level::INFO>(m.c_str());
    std::cerr << "3:OccCommand::openDevice: calling open device=" << devicePath.c_str() << std::endl << std::flush;
    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        const int open_errno = errno;
        log<level::ERR>("OccCommand::openDevice: open failed", entry("ERRNO=%d", open_errno));
        // This would log and terminate since its not handled.
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_ERRNO(open_errno),
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    else
        log<level::INFO>("OccCommand::openDevice: open success");

    return;
}

void OccCommand::closeDevice()
{
    using namespace phosphor::logging;
    if (fd >= 0)
    {
        log<level::INFO>("OccCommand::closeDevice: calling close()");
        close(fd);
        fd = -1;
    }
}

std::vector<int32_t> OccCommand::send(std::vector<int32_t> command)
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    std::vector<int32_t> response{};

    log<level::DEBUG>("OccCommand::send: calling openDevice()");
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
    log<level::DEBUG>("OccCommand::send: calling write()");
    auto rc = write(fd, cmdInBytes.data(), size);
    if (rc < 0 || (rc != size))
    {
        const int write_errno = errno;
        log<level::ERR>("OccCommand::send: write failed");
        // This would log and terminate since its not handled.
        elog<WriteFailure>(
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_ERRNO(write_errno),
            phosphor::logging::org::open_power::OCC::Device::WriteFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    else
    {
        log<level::DEBUG>("OccCommand::send: write succeeded");
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
            log<level::DEBUG>("OccCommand::send: read completed");
            // We have read all that we can.
            break;
        }
        else
        {
            log<level::ERR>("OccCommand::send: read failed");
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
void OccCommand::activeStatusEvent(sdbusplus::message::message& msg)
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

            std::string m = "OccCommand::activeStatusEvent: " + devicePath + " is ACTIVE";
            log<level::INFO>(m.c_str());
        }
        else
        {
            occActive = false;

            std::string m = "OccCommand::activeStatusEvent: " + devicePath + " is DISABLED";
            log<level::INFO>(m.c_str());

            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
