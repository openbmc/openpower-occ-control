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
        if (value)
        {
            log<level::INFO>(
                fmt::format("Status::occActive OCC{} changed to true", instance)
                    .c_str());
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
            log<level::INFO>(">>Status::occActive changed to false");
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
    using namespace phosphor::logging;
#ifdef PLDM
    log<level::INFO>(">>Status::resetOCC(PLDM)");
    if (resetCallBack)
    {
        this->resetCallBack(instance);
    }
#else
    log<level::INFO>(">>Status::resetOCC()");
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

} // namespace occ
} // namespace open_power
