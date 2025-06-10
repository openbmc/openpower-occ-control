#include "config.h"

#include "occ_pass_through.hpp"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/lg2.hpp>

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
    const char* path,
    std::unique_ptr<open_power::occ::powermode::PowerMode>& powerModeRef) :
    Iface(utils::getBus(), path), path(path), pmode(powerModeRef),
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

    if (!occActive)
    {
        lg2::error(
            "PassThrough::send() - OCC{INST} not active, command not sent",
            "INST", occInstance);
        return response;
    }

    if (command.size() >= 3)
    {
        const uint16_t dataLen = command[1] << 8 | command[2];
        std::string dataString = "";
        if (command.size() > 3)
        {
            // Trace first 4 bytes of command data
            size_t index = 3;
            dataString = "0x";
            for (; (index < 7) && (index < command.size()); ++index)
            {
                dataString += std::format("{:02X}", command[index]);
            }
            if (index < command.size())
            {
                dataString += "...";
            }
        }
        lg2::info(
            "PassThrough::send() Sending {CMD} command to OCC{INST} (data len={LEN}, data={DATA})",
            "CMD", lg2::hex, command.front(), "INST", occInstance, "LEN",
            dataLen, "DATA", dataString);
    }
    else
    {
        lg2::info("PassThrough::send() Sending {CMD} command to OCC{INST}",
                  "CMD", command.front(), "INST", occInstance);
    }
    CmdStatus status = occCmd.send(command, response);
    if (status == CmdStatus::SUCCESS)
    {
        if (response.size() >= 5)
        {
            lg2::debug("PassThrough::send() response had {LEN} bytes", "LEN",
                       response.size());
        }
        else
        {
            lg2::error("PassThrough::send() Invalid OCC response");
            dump_hex(response);
        }
    }
    else
    {
        lg2::error(
            "PassThrough::send(): OCC command failed with status {STATUS}",
            "STATUS", status);
    }

    return response;
}

bool PassThrough::setMode(const uint8_t mode, const uint16_t modeData)
{
    SysPwrMode newMode = SysPwrMode(mode);

    if (!pmode)
    {
        lg2::error("PassThrough::setMode: PowerMode is not defined!");
        return false;
    }

    if (!pmode->isValidMode(SysPwrMode(mode)))
    {
        lg2::error(
            "PassThrough::setMode() Unsupported mode {MODE} requested ({DATA})",
            "MODE", newMode, "DATA", modeData);
        return false;
    }

    if (((newMode == SysPwrMode::FFO) || (newMode == SysPwrMode::SFP)) &&
        (modeData == 0))
    {
        lg2::error(
            "PassThrough::setMode() Mode {MODE} requires non-zero frequency point.",
            "MODE", newMode);
        return false;
    }

    lg2::info("PassThrough::setMode() Setting Power Mode {MODE} (data: {DATA})",
              "MODE", uint8_t(newMode), "DATA", modeData);
    return pmode->setMode(newMode, modeData);
}

// Called at OCC Status change signal
void PassThrough::activeStatusEvent(sdbusplus::message_t& msg)
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
