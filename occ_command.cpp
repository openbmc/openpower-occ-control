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
#include <fmt/core.h>

//#define TRACE_PACKETS

namespace open_power
{
namespace occ
{


// Trace block of data in hex
void dump_hex(const std::vector<std::int32_t> data, const unsigned int data_len)
{
    using namespace phosphor::logging;

    char line[64] = {0};
    char *c = line;
    unsigned int dump_length = data.size();
    if ((data_len > 0) && (data_len < dump_length)) dump_length = data_len;
    for (uint32_t i = 0; i < dump_length; i++)
    {
        if (i % 16 == 0)
        {
            sprintf(c, "%04X: ", i);
            c += 6;
        }
        else if (i % 4 == 0)
        {
            c[0] = ' ';
            c++;
        }

        sprintf(c, "%02X", data.at(i));
        c += 2;

        if ((i % 16 == 15) || (i == dump_length-1))
        {
            c[0] = '\0';
            log<level::INFO>(line);
            c = line;
        }
    }
}


OccCommand::OccCommand(uint8_t instance, sdbusplus::bus::bus& bus, const char* path) :
    occ_instance(instance),
    path(path),
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    activeStatusSignal(
        bus, sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&OccCommand::activeStatusEvent), this,
                  std::placeholders::_1))
{
    using namespace phosphor::logging;
    log<level::INFO>(fmt::format("OccCommand::OccCommand(path={}, devicePath={}",
                                 this->path, devicePath).c_str());

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

    log<level::DEBUG>(fmt::format("OccCommand::openDevice: calling open {}", devicePath).c_str());
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
        log<level::DEBUG>("OccCommand::openDevice: open success");

    return;
}

void OccCommand::closeDevice()
{
    using namespace phosphor::logging;
    if (fd >= 0)
    {
        log<level::DEBUG>("OccCommand::closeDevice: calling close()");
        close(fd);
        fd = -1;
    }
}

CmdStatus OccCommand::send(std::vector<int32_t> command,
                           std::vector<int32_t> & response)
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;
    CmdStatus status = FAILURE;

    response.clear();

    log<level::DEBUG>("OccCommand::send: calling openDevice()");
    openDevice();

    if (fd < 0)
    {
        // OCC is inactive; empty response
        return OPEN_FAILURE;
    }

    // OCC only understands [bytes] so need array of bytes. Doing this
    // because rest-server currently treats all int* as 32 bit integer.
    std::vector<uint8_t> cmdInBytes;
    cmdInBytes.resize(command.size());

    // Populate uint8_t version of vector.
    std::transform(command.begin(), command.end(), cmdInBytes.begin(),
                   [](decltype(cmdInBytes)::value_type x) { return x; });

    ssize_t size = cmdInBytes.size() * sizeof(decltype(cmdInBytes)::value_type);
#ifdef TRACE_PACKETS
    uint8_t cmd_type = command[0];
    log<level::INFO>(fmt::format("OCC{}: Sending 0x{:02X} command (length={}, {})",
                                 occ_instance, cmd_type, command.size(), devicePath).c_str());
    dump_hex(command);
#else
    log<level::DEBUG>("OccCommand::send: calling write()");
#endif
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
            status = SUCCESS;
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

#ifdef TRACE_PACKETS
    if (response.size() > 0)
    {
        log<level::INFO>(fmt::format("OCC{}: Received 0x{:02X} response (length={} w/checksum)",
                                     occ_instance, cmd_type, response.size()).c_str());
        dump_hex(response, 64);
    }
#endif

    closeDevice();

    return status;
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

            log<level::INFO>(fmt::format("OccCommand::activeStatusEvent: {} is ACTIVE",
                                         devicePath).c_str());
        }
        else
        {
            occActive = false;

            log<level::INFO>(fmt::format("OccCommand::activeStatusEvent: {} is DISABLED",
                                         devicePath).c_str());

            this->closeDevice();
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
