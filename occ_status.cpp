#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "utils.hpp"

#include <fmt/core.h>

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
void Status::deviceErrorHandler(bool error)
{
    // Make sure we have an error
    if (error)
    {
        // This would deem OCC inactive
        this->occActive(false);

        // Reset the OCC
        this->resetOCC();
    }
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
    auto service = getService(bus, CONTROL_HOST_PATH, CONTROL_HOST_INTF);

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
    using namespace phosphor::logging;

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

            if ((state == 0x03) && (device.master()))
            {
                // Kernel detected that the master OCC went to active state
                occsWentActive();
            }
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

// Special processing that needs to happen once the OCCs change to ACTIVE state
void Status::occsWentActive()
{
    CmdStatus status = CmdStatus::SUCCESS;

    status = sendModeChange();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(fmt::format("Status::occsWentActive: OCC mode "
                                    "change failed with status {}",
                                    status)
                            .c_str());
    }

    status = sendIpsData();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "Status::occsWentActive: Sending Idle Power Save Config data"
                " failed with status {}",
                status)
                .c_str());
    }
}

// Send mode change request to the master OCC
CmdStatus Status::sendModeChange()
{
    CmdStatus status = CmdStatus::FAILURE;

    if (device.master())
    {
        log<level::INFO>(
            fmt::format(
                "Status::sendModeChange: Sending CHANGE_MODE to master OCC{}",
                instance)
                .c_str());

        std::vector<std::uint8_t> cmd, rsp;
        cmd.push_back(0x20); // Command (SET_MODE_AND_STATE)
        cmd.push_back(0x00); // Data Length (2 bytes)
        cmd.push_back(0x06);
        cmd.push_back(0x30); // Data (Version)
        cmd.push_back(0x00); // State (no change)
        cmd.push_back(0x0C); // Mode (TODO: read from saved data)
        cmd.push_back(0x00); // Mode Data (Freq Point)
        cmd.push_back(0x00); //
        cmd.push_back(0x00); // reserved
        log<level::INFO>(
            fmt::format("Status::sendModeChange: SET_MODE command ({} bytes)",
                        cmd.size())
                .c_str());
        status = occCmd.send(cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() == 5)
            {
                if (rsp[1] == 0x20) // rsp command (SET MODE AND STATE)
                {
                    if (rsp[2] == 0x00) // rsp status (SUCCESS)
                    {
                        log<level::INFO>(
                            "Status::sendModeChange: - Mode change "
                            "completed successfully");
                    }
                    else
                    {
                        log<level::ERR>(
                            fmt::format("Status::sendModeChange: SET MODE "
                                        "failed with status 0x{:02X}",
                                        rsp[2])
                                .c_str());
                    }
                }
                else
                {
                    log<level::ERR>(
                        fmt::format("Status::sendModeChange: SET MODE response "
                                    "command mismatch"
                                    " (received 0x{:02x}, expected 0x20)",
                                    rsp[1])
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
            fmt::format("Status::sendModeChange: MODE CHANGE does not "
                        "get sent to slave OCC{}",
                        instance)
                .c_str());
    }
    return status;
}

// Send Idle Power Saver config data to the master OCC
CmdStatus Status::sendIpsData()
{
    CmdStatus status = CmdStatus::FAILURE;

    if (device.master())
    {
        log<level::INFO>(
            fmt::format(
                "Status::sendIpsData: Sending default IPS data to master OCC{}",
                instance)
                .c_str());

        std::vector<std::uint8_t> cmd, rsp;
        cmd.push_back(0x21); // Command (SEND_CFG_DATA)
        cmd.push_back(0x00); // Data Length (2 bytes)
        cmd.push_back(0x09);
        // Data:
        cmd.push_back(0x11); // Config Format: IPS Settings
        cmd.push_back(0x00); // Version
        cmd.push_back(0x01); // IPS Enable: enabled
        cmd.push_back(0x00); // Enter Delay Time (240s)
        cmd.push_back(0xF0); //
        cmd.push_back(0x08); // Enter Utilization (8%)
        cmd.push_back(0x00); // Exit Delay Time (10s)
        cmd.push_back(0x0A); //
        cmd.push_back(0x0C); // Exit Utilization (12%)
        log<level::INFO>(
            fmt::format(
                "Status::sendIpsData: SET_CFG_DATA[IPS] command ({} bytes)",
                cmd.size())
                .c_str());
        status = occCmd.send(cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() == 5)
            {
                if (rsp[1] == 0x21) // rsp command (SET CONFIG DATA)
                {
                    if (rsp[2] == 0x00) // rsp status (SUCCESS)
                    {
                        log<level::INFO>(
                            "Status::sendIpsData: - SET_CFG_DATA[IPS] "
                            "completed successfully");
                    }
                    else
                    {
                        log<level::ERR>(
                            fmt::format(
                                "Status::sendIpsData: SET_CFG_DATA[IPS] "
                                "failed with status 0x{:02X}",
                                rsp[2])
                                .c_str());
                    }
                }
                else
                {
                    log<level::ERR>(
                        fmt::format(
                            "Status::sendIpsData: SET_CFG_DATA[IPS] response "
                            "command mismatch"
                            " (received 0x{:02x}, expected 0x21)",
                            rsp[1])
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
                log<level::ERR>(
                    "Status::sendIpsData: SET_CFG_DATA[IPS] FAILED!");
            }
        }
    }
    else
    {
        log<level::ERR>(
            fmt::format("Status::sendIpsData: SET_CFG_DATA[IPS] does not "
                        "get sent to slave OCC{}",
                        instance)
                .c_str());
    }
    return status;
}
} // namespace occ
} // namespace open_power
