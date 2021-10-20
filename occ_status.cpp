#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <fmt/core.h>

#ifdef POWER10
#include <com/ibm/Host/Target/server.hpp>
#endif
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value != this->occActive())
    {
        log<level::INFO>(fmt::format("Status::occActive OCC{} changed to {}",
                                     instance, value)
                             .c_str());
        if (value)
        {
            // Bind the device
            device.bind();

            // Start watching for errors
            addErrorWatch();

            // Reset last OCC state
            lastState = 0;

            // Call into Manager to let know that we have bound
            if (this->callBack)
            {
                this->callBack(value);
            }
        }
        else
        {
            // Call into Manager to let know that we will unbind.
            if (this->callBack)
            {
                this->callBack(value);
            }

            // Stop watching for errors
            removeErrorWatch();

            // Do the unbind.
            device.unBind();
        }
    }
    else if (value && !device.bound())
    {
        // Existing error watch is on a dead file descriptor.
        removeErrorWatch();

        /*
         * In it's constructor, Status checks Device::bound() to see if OCC is
         * active or not.
         * Device::bound() checks for occX-dev0 directory.
         * We will lose occX-dev0 directories during FSI rescan.
         * So, if we start this application (and construct Status), and then
         * later do FSI rescan, we will end up with occActive = true and device
         * NOT bound. Lets correct that situation here.
         */
        device.bind();

        // Add error watch again
        addErrorWatch();
    }
    else if (!value && device.bound())
    {
        removeErrorWatch();

        // In the event that the application never receives the active signal
        // even though the OCC is active (this can occur if the BMC is rebooted
        // with the host on, since the initial OCC driver probe will discover
        // the OCCs), this application needs to be able to unbind the device
        // when we get the OCC inactive signal.
        device.unBind();
    }
    return Base::Status::occActive(value);
}

// Callback handler when a device error is reported.
void Status::deviceError()
{
    // This would deem OCC inactive
    this->occActive(false);

    // Reset the OCC
    this->resetOCC();
}

// Sends message to host control command handler to reset OCC
void Status::resetOCC()
{
    log<level::INFO>(
        fmt::format(">>Status::resetOCC() - requesting reset for OCC{}",
                    instance)
            .c_str());
#ifdef PLDM
    if (resetCallBack)
    {
        this->resetCallBack(instance);
    }
#else
    constexpr auto CONTROL_HOST_PATH = "/org/open_power/control/host0";
    constexpr auto CONTROL_HOST_INTF = "org.open_power.Control.Host";

    // This will throw exception on failure
    auto service = utils::getService(CONTROL_HOST_PATH, CONTROL_HOST_INTF);

    auto& bus = utils::getBus();
    auto method = bus.new_method_call(service.c_str(), CONTROL_HOST_PATH,
                                      CONTROL_HOST_INTF, "Execute");
    // OCC Reset control command
    method.append(convertForMessage(Control::Host::Command::OCCReset).c_str());

    // OCC Sensor ID for callout reasons
    method.append(std::variant<uint8_t>(std::get<0>(sensorMap.at(instance))));
    bus.call_noreply(method);
    return;
#endif
}

// Handler called by Host control command handler to convey the
// status of the executed command
void Status::hostControlEvent(sdbusplus::message::message& msg)
{
    std::string cmdCompleted{};
    std::string cmdStatus{};

    msg.read(cmdCompleted, cmdStatus);

    log<level::DEBUG>("Host control signal values",
                      entry("COMMAND=%s", cmdCompleted.c_str()),
                      entry("STATUS=%s", cmdStatus.c_str()));

    if (Control::Host::convertResultFromString(cmdStatus) !=
        Control::Host::Result::Success)
    {
        if (Control::Host::convertCommandFromString(cmdCompleted) ==
            Control::Host::Command::OCCReset)
        {
            // Must be a Timeout. Log an Error trace
            log<level::ERR>(
                "Error resetting the OCC.", entry("PATH=%s", path.c_str()),
                entry("SENSORID=0x%X", std::get<0>(sensorMap.at(instance))));
        }
    }
    return;
}

void Status::readOccState()
{
    unsigned int state;
    const fs::path filename =
        fs::path(DEV_PATH) /
        fs::path(sysfsName + "." + std::to_string(instance + 1)) / "occ_state";

    log<level::DEBUG>(
        fmt::format("Status::readOccState: reading OCC{} state from {}",
                    instance, filename.c_str())
            .c_str());

    std::ifstream file(filename, std::ios::in);
    const int open_errno = errno;
    if (file)
    {
        file >> state;
        if (state != lastState)
        {
            // Trace OCC state changes
            log<level::INFO>(
                fmt::format("Status::readOccState: OCC{} state 0x{:02X}",
                            instance, state)
                    .c_str());
            lastState = state;

#ifdef POWER10
            if ((OccState(state) == OccState::ACTIVE) && (device.master()))
            {
                // Kernel detected that the master OCC went to active state
                occsWentActive();
            }
#endif
        }
        file.close();
    }
    else
    {
        // If not able to read, OCC may be offline
        log<level::DEBUG>(
            fmt::format("Status::readOccState: open failed (errno={})",
                        open_errno)
                .c_str());
        lastState = 0;
    }
}

#ifdef POWER10
// Check if Hypervisor target is PowerVM
bool Status::isPowerVM()
{
    using namespace open_power::occ::powermode;
    namespace Hyper = sdbusplus::com::ibm::Host::server;
    constexpr auto HYPE_PATH = "/com/ibm/host0/hypervisor";
    constexpr auto HYPE_INTERFACE = "com.ibm.Host.Target";
    constexpr auto HYPE_PROP = "Target";

    bool powerVmTarget = false;

    // This will throw exception on failure
    auto& bus = utils::getBus();
    auto service = utils::getService(HYPE_PATH, HYPE_INTERFACE);
    auto method = bus.new_method_call(service.c_str(), HYPE_PATH,
                                      "org.freedesktop.DBus.Properties", "Get");
    method.append(HYPE_INTERFACE, HYPE_PROP);
    auto reply = bus.call(method);

    std::variant<std::string> hyperEntryValue;
    reply.read(hyperEntryValue);
    auto propVal = std::get<std::string>(hyperEntryValue);
    if (Hyper::Target::convertHypervisorFromString(propVal) ==
        Hyper::Target::Hypervisor::PowerVM)
    {
        powerVmTarget = true;
    }

    log<level::DEBUG>(
        fmt::format("Status::isPowerVM returning {}", powerVmTarget).c_str());

    return powerVmTarget;
}

// Get the requested power mode
SysPwrMode Status::getMode()
{
    using namespace open_power::occ::powermode;
    SysPwrMode pmode = SysPwrMode::NO_CHANGE;

    // This will throw exception on failure
    auto& bus = utils::getBus();
    auto service = utils::getService(PMODE_PATH, PMODE_INTERFACE);
    auto method = bus.new_method_call(service.c_str(), PMODE_PATH,
                                      "org.freedesktop.DBus.Properties", "Get");
    method.append(PMODE_INTERFACE, POWER_MODE_PROP);
    auto reply = bus.call(method);

    std::variant<std::string> stateEntryValue;
    reply.read(stateEntryValue);
    auto propVal = std::get<std::string>(stateEntryValue);
    pmode = powermode::convertStringToMode(propVal);

    log<level::DEBUG>(
        fmt::format("Status::getMode returning {}", pmode).c_str());

    return pmode;
}

// Special processing that needs to happen once the OCCs change to ACTIVE state
void Status::occsWentActive()
{
    CmdStatus status = CmdStatus::SUCCESS;

    status = sendModeChange();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "Status::occsWentActive: OCC mode change failed with status {}",
                status)
                .c_str());
    }

    status = sendIpsData();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "Status::occsWentActive: Sending Idle Power Save Config data failed with status {}",
                status)
                .c_str());
    }
}

// Send mode change request to the master OCC
CmdStatus Status::sendModeChange()
{
    CmdStatus status = CmdStatus::FAILURE;

    if (!device.master())
    {
        log<level::ERR>(
            fmt::format(
                "Status::sendModeChange: MODE CHANGE does not get sent to slave OCC{}",
                instance)
                .c_str());
        return status;
    }
    if (!isPowerVM())
    {
        // Mode change is only supported on PowerVM systems
        log<level::DEBUG>(
            "Status::sendModeChange: MODE CHANGE does not get sent on non-PowerVM systems");
        return CmdStatus::SUCCESS;
    }

    const SysPwrMode newMode = getMode();

    if (VALID_POWER_MODE_SETTING(newMode))
    {
        std::vector<std::uint8_t> cmd, rsp;
        cmd.push_back(uint8_t(CmdType::SET_MODE_AND_STATE));
        cmd.push_back(0x00); // Data Length (2 bytes)
        cmd.push_back(0x06);
        cmd.push_back(0x30); // Data (Version)
        cmd.push_back(uint8_t(OccState::NO_CHANGE));
        cmd.push_back(uint8_t(newMode));
        cmd.push_back(0x00); // Mode Data (Freq Point)
        cmd.push_back(0x00); //
        cmd.push_back(0x00); // reserved
        log<level::INFO>(
            fmt::format(
                "Status::sendModeChange: SET_MODE({}) command to OCC{} ({} bytes)",
                newMode, instance, cmd.size())
                .c_str());
        status = occCmd.send(cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() == 5)
            {
                if (RspStatus::SUCCESS == RspStatus(rsp[2]))
                {
                    log<level::DEBUG>(
                        "Status::sendModeChange: - Mode change completed successfully");
                }
                else
                {
                    log<level::ERR>(
                        fmt::format(
                            "Status::sendModeChange: SET MODE failed with status 0x{:02X}",
                            rsp[2])
                            .c_str());
                }
            }
            else
            {
                log<level::ERR>(
                    "Status::sendModeChange: INVALID SET MODE response");
                dump_hex(rsp);
            }
        }
        else
        {
            if (status == CmdStatus::OPEN_FAILURE)
            {
                log<level::WARNING>(
                    "Status::sendModeChange: OCC not active yet");
            }
            else
            {
                log<level::ERR>("Status::sendModeChange: SET_MODE FAILED!");
            }
        }
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "Status::sendModeChange: Unable to set power mode to {}",
                newMode)
                .c_str());
    }

    return status;
}

// Send Idle Power Saver config data to the master OCC
CmdStatus Status::sendIpsData()
{
    CmdStatus status = CmdStatus::FAILURE;

    if (!device.master())
    {
        log<level::ERR>(
            fmt::format(
                "Status::sendIpsData: SET_CFG_DATA[IPS] does not get sent to slave OCC{}",
                instance)
                .c_str());
        return status;
    }
    if (!isPowerVM())
    {
        // Idle Power Saver data is only supported on PowerVM systems
        log<level::DEBUG>(
            "Status::sendIpsData: SET_CFG_DATA[IPS] does not get sent on non-PowerVM systems");
        return CmdStatus::SUCCESS;
    }

    std::vector<std::uint8_t> cmd, rsp;
    cmd.push_back(uint8_t(CmdType::SET_CONFIG_DATA));
    cmd.push_back(0x00); // Data Length (2 bytes)
    cmd.push_back(0x09);
    // Data:
    cmd.push_back(0x11); // Config Format: IPS Settings
    cmd.push_back(0x00); // Version
    cmd.push_back(0x00); // IPS Enable: disabled
    cmd.push_back(0x00); // Enter Delay Time (240s)
    cmd.push_back(0xF0); //
    cmd.push_back(0x08); // Enter Utilization (8%)
    cmd.push_back(0x00); // Exit Delay Time (10s)
    cmd.push_back(0x0A); //
    cmd.push_back(0x0C); // Exit Utilization (12%)
    log<level::INFO>(
        fmt::format(
            "Status::sendIpsData: SET_CFG_DATA[IPS] command to OCC{} ({} bytes)",
            instance, cmd.size())
            .c_str());
    status = occCmd.send(cmd, rsp);
    if (status == CmdStatus::SUCCESS)
    {
        if (rsp.size() == 5)
        {
            if (RspStatus::SUCCESS == RspStatus(rsp[2]))
            {
                log<level::DEBUG>(
                    "Status::sendIpsData: - SET_CFG_DATA[IPS] completed successfully");
            }
            else
            {
                log<level::ERR>(
                    fmt::format(
                        "Status::sendIpsData: SET_CFG_DATA[IPS] failed with status 0x{:02X}",
                        rsp[2])
                        .c_str());
            }
        }
        else
        {
            log<level::ERR>(
                "Status::sendIpsData: INVALID SET_CFG_DATA[IPS] response");
            dump_hex(rsp);
        }
    }
    else
    {
        if (status == CmdStatus::OPEN_FAILURE)
        {
            log<level::WARNING>("Status::sendIpsData: OCC not active yet");
        }
        else
        {
            log<level::ERR>("Status::sendIpsData: SET_CFG_DATA[IPS] FAILED!");
        }
    }

    return status;
}

#endif // POWER10

} // namespace occ
} // namespace open_power
