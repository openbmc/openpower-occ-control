#include "config.h"

#include "occ_command.hpp"

#include "elog-errors.hpp"

#include <errno.h>
#include <fcntl.h>
#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <string>

//#define TRACE_PACKETS

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

// Trace block of data in hex
void dump_hex(const std::vector<std::uint8_t>& data,
              const unsigned int data_len)
{
    unsigned int dump_length = data.size();
    if ((data_len > 0) && (data_len < dump_length))
    {
        dump_length = data_len;
    }
    std::string s;
    for (uint32_t i = 0; i < dump_length; i++)
    {
        if (i % 16 == 0)
        {
            s += fmt::format("{:04X}: ", i);
        }
        else if (i % 4 == 0)
        {
            s += " ";
        }

        s += fmt::format("{:02X}", data.at(i));

        if ((i % 16 == 15) || (i == (dump_length - 1)))
        {
            log<level::INFO>(s.c_str());
            s.clear();
        }
    }
}

OccCommand::OccCommand(uint8_t instance, sdbusplus::bus::bus& bus,
                       const char* path) :
    occInstance(instance),
    path(path),
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    activeStatusSignal(
        bus, sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&OccCommand::activeStatusEvent), this,
                  std::placeholders::_1))
{
    log<level::DEBUG>(
        fmt::format("OccCommand::OccCommand(path={}, devicePath={}", this->path,
                    devicePath)
            .c_str());
}

void OccCommand::openDevice()
{
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    log<level::DEBUG>(
        fmt::format("OccCommand::openDevice: calling open {}", devicePath)
            .c_str());
    fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        const int open_errno = errno;
        log<level::ERR>(
            fmt::format(
                "OccCommand::openDevice: open failed (errno={}, path={})",
                open_errno, devicePath)
                .c_str());
        // This would log and terminate since its not handled.
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_ERRNO(open_errno),
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    else
    {
        log<level::DEBUG>("OccCommand::openDevice: open success");
    }

    return;
}

void OccCommand::closeDevice()
{
    if (fd >= 0)
    {
        log<level::DEBUG>("OccCommand::closeDevice: calling close()");
        close(fd);
        fd = -1;
    }
}

CmdStatus OccCommand::send(const std::vector<uint8_t>& command,
                           std::vector<uint8_t>& response)
{
    using namespace sdbusplus::org::open_power::OCC::Device::Error;
    CmdStatus status = CmdStatus::FAILURE;

    response.clear();

    log<level::DEBUG>("OccCommand::send: calling openDevice()");
    openDevice();

    if (fd < 0)
    {
        // OCC is inactive; empty response
        return CmdStatus::OPEN_FAILURE;
    }

#ifdef TRACE_PACKETS
    uint8_t cmd_type = command[0];
    log<level::INFO>(
        fmt::format("OCC{}: Sending 0x{:02X} command (length={}, {})",
                    occInstance, cmd_type, command.size(), devicePath)
            .c_str());
    dump_hex(command);
#else
    log<level::DEBUG>("OccCommand::send: calling write()");
#endif

    int retries = 1; // Allow a retry if a command fails to get valid response
    do
    {
        auto rc = write(fd, command.data(), command.size());
        if ((rc < 0) || (rc != (int)command.size()))
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
                status = CmdStatus::SUCCESS;
                break;
            }
            else
            {
                log<level::ERR>("OccCommand::send: read failed");
                // This would log and terminate since its not handled.
                elog<ReadFailure>(
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_ERRNO(read_errno),
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_DEVICE_PATH(devicePath.c_str()));
            }
        }

        if (response.size() > 2)
        {
#ifdef TRACE_PACKETS
            log<level::INFO>(
                fmt::format(
                    "OCC{}: Received 0x{:02X} response (length={} w/checksum)",
                    occInstance, cmd_type, response.size())
                    .c_str());
            dump_hex(response, 64);
#endif

            // Validate checksum (last 2 bytes of response)
            const unsigned int csumIndex = response.size() - 2;
            const uint32_t rspChecksum =
                (response[csumIndex] << 8) + response[csumIndex + 1];
            uint32_t calcChecksum = 0;
            for (unsigned int index = 0; index < csumIndex; ++index)
            {
                calcChecksum += response[index];
            }
            while (calcChecksum > 0xFFFF)
            {
                calcChecksum = (calcChecksum & 0xFFFF) + (calcChecksum >> 16);
            }
            if (calcChecksum != rspChecksum)
            {
                log<level::ERR>(
                    fmt::format("OCC{}: Checksum Mismatch: response "
                                "0x{:04X}, calculated 0x{:04X}",
                                occInstance, rspChecksum, calcChecksum)
                        .c_str());
                dump_hex(response);
                status = CmdStatus::INVALID_CHECKSUM;
            }
            else
            {
                // Validate response was for the specified command
                if (command[0] == response[1])
                {
                    // Valid response received

                    // Strip off 2 byte checksum
                    response.pop_back();
                    response.pop_back();
                    break;
                }
                else
                {
                    log<level::ERR>(
                        fmt::format(
                            "OccCommand::send: Response command mismatch "
                            "(sent: "
                            "0x{:02X}, rsp: 0x{:02X}, rsp seq#: 0x{:02X}",
                            command[0], response[1], response[0])
                            .c_str());
                    dump_hex(response, 64);
                }
            }
        }
        else
        {
            log<level::ERR>(
                fmt::format(
                    "OccCommand::send: Invalid OCC{} response length: {}",
                    occInstance, response.size())
                    .c_str());
            status = CmdStatus::FAILURE;
            dump_hex(response);
        }

        if (retries > 0)
        {
            log<level::ERR>("OccCommand::send: Command will be retried");
            response.clear();
        }

    } while (retries-- > 0);

    closeDevice();

    return status;
}

// Called at OCC Status change signal
void OccCommand::activeStatusEvent(sdbusplus::message::message& msg)
{
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
