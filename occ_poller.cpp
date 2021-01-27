#include "occ_poller.hpp"

#include "elog-errors.hpp"
#include "occ_manager.hpp"

#include <errno.h>
#include <fcntl.h>
#include <fmt/core.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

void Poller::startPoller()
{
    log<level::DEBUG>(
        fmt::format(
            "Poller::startPoller(): OCC{} will be polled every {} seconds",
            occ_instance, interval)
            .c_str());

    // Start timer
    _pollTimer->restartOnce(std::chrono::seconds(interval));
}

void Poller::pollerTimerExpired()
{
    if (manager.getNumOCCs() > 0)
    {
        if (!_pollTimer)
        {
            log<level::ERR>(
                "Poller::pollerTimerExpired() ERROR: Timer not defined");
            return;
        }

        // Send Poll
        log<level::DEBUG>(
            fmt::format("Poller::pollerTimerExpired() Send POLL to OCC{}",
                        occ_instance)
                .c_str());
        // POLL command
        std::vector<std::uint8_t> cmd{0, 0, 1, 0x20}, rsp;
        CmdStatus status = manager.sendOccCommand(occ_instance, cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() >= 5)
            {
                log<level::DEBUG>(fmt::format("Poller::pollerTimerExpired() "
                                              "POLL response had {} bytes",
                                              rsp.size())
                                      .c_str());
            }
            else
            {
                log<level::ERR>(
                    "Poller::pollerTimerExpired() Invalid POLL response");
                dump_hex(rsp);
            }
        }
        else
        {
            if (status == CmdStatus::OPEN_FAILURE)
                log<level::WARNING>(
                    "Poller::pollerTimerExpired() - OCC not active yet");
            else
                log<level::ERR>(
                    "Poller::pollerTimerExpired() - POLL command failed!");
        }

        // Restart Timer
        _pollTimer->restartOnce(std::chrono::seconds(interval));
    }
    else
    {
        log<level::INFO>("Poller::pollerTimerExpire() - No OCCs found");
    }
}

// Reads the occs_present file and analyzes the data
void Poller::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    // Get the number of bytes to read
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        log<level::INFO>(
            fmt::format("Poller::analyzeEvent: OCC State Change Detected "
                        "(failing ioctl: rc={})",
                        r)
                .c_str());
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_DEVICE_PATH(file.c_str()));
    }

    auto data = readFile(len);
    if (data.empty())
    {
        log<level::ERR>(
            "Poller::analyzeEvent: OCC State Change Detected (empty data)");
        return;
    }

    if (callBack)
    {
        callBack(true);
    }
}

} // namespace occ
} // namespace open_power
