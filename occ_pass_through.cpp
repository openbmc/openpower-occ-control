#include "config.h"

#include "occ_pass_through.hpp"

#include "elog-errors.hpp"

#include <errno.h>
#include <fcntl.h>
#include <fmt/core.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace sdbusplus::org::open_power::OCC::Device::Error;

PassThrough::PassThrough(
    const char* path
#ifdef POWER10
    ,
    std::unique_ptr<open_power::occ::powermode::PowerMode>& powerModeRef
#endif
    ) :
    Iface(utils::getBus(), path),
    path(path),
#ifdef POWER10
    pmode(powerModeRef),
#endif
    devicePath(OCC_DEV_PATH + std::to_string((this->path.back() - '0') + 1)),
    occInstance(this->path.back() - '0'),
    activeStatusSignal(
        utils::getBus(),
        sdbusRule::propertiesChanged(path, "org.open_power.OCC.Status"),
        std::bind(std::mem_fn(&PassThrough::activeStatusEvent), this,
                  std::placeholders::_1)),
    occCmd(occInstance, path)
{
    // Nothing to do.
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    std::vector<int32_t> response{};

    // OCC only understands [bytes] so need array of bytes. Doing this
    // because rest-server currently treats all int* as 32 bit integer.
    std::vector<uint8_t> cmdInBytes, rsp;
    cmdInBytes.resize(command.size());

    // Populate uint8_t version of vector.
    std::transform(command.begin(), command.end(), cmdInBytes.begin(),
                   [](decltype(cmdInBytes)::value_type x) { return x; });

    rsp = send(cmdInBytes);

    response.resize(rsp.size());
    std::transform(rsp.begin(), rsp.end(), response.begin(),
                   [](decltype(response)::value_type x) { return x; });

    return response;
}

std::vector<uint8_t> PassThrough::send(std::vector<uint8_t> command)
{
    std::vector<uint8_t> response{};

    log<level::INFO>(
        fmt::format("PassThrough::send() Sending 0x{:02X} command to OCC{}",
                    command.front(), occInstance)
            .c_str());
    CmdStatus status = occCmd.send(command, response);
    if (status == CmdStatus::SUCCESS)
    {
        if (response.size() >= 5)
        {
            log<level::DEBUG>(
                fmt::format("PassThrough::send() response had {} bytes",
                            response.size())
                    .c_str());
        }
        else
        {
            log<level::ERR>("PassThrough::send() Invalid OCC response");
            dump_hex(response);
        }
    }
    else
    {
        if (status == CmdStatus::OPEN_FAILURE)
        {
            log<level::WARNING>("PassThrough::send() - OCC not active yet");
        }
        else
        {
            log<level::ERR>(
                fmt::format(
                    "PassThrough::send(): OCC command failed with status={}",
                    status)
                    .c_str());
        }
    }

    return response;
}

bool PassThrough::setMode(const uint8_t mode, const uint16_t modeData)
{
#ifdef POWER10
    SysPwrMode newMode = SysPwrMode(mode);

    if ((!VALID_POWER_MODE_SETTING(newMode)) &&
        (!VALID_OEM_POWER_MODE_SETTING(newMode)))
    {
        log<level::ERR>(
            fmt::format(
                "PassThrough::setMode() Unsupported mode {} requested (0x{:04X})",
                newMode, modeData)
                .c_str());
        return false;
    }

    if (((newMode == SysPwrMode::FFO) || (newMode == SysPwrMode::SFP)) &&
        (modeData == 0))
    {
        log<level::ERR>(
            fmt::format(
                "PassThrough::setMode() Mode {} requires non-zero frequency point.",
                newMode)
                .c_str());
        return false;
    }

    if (!pmode)
    {
        log<level::ERR>("PassThrough::setMode: PowerMode is not defined!");
        return false;
    }

    log<level::INFO>(
        fmt::format("PassThrough::setMode() Setting Power Mode {} (data: {})",
                    newMode, modeData)
            .c_str());
    return pmode->setMode(newMode, modeData);
#else
    log<level::DEBUG>(
        fmt::format(
            "PassThrough::setMode() No support to setting Power Mode {} (data: {})",
            mode, modeData)
            .c_str());
    return false;
#endif
}

// Called at OCC Status change signal
void PassThrough::activeStatusEvent(sdbusplus::message::message& msg)
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
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
