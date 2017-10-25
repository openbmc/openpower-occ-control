#include <phosphor-logging/log.hpp>
#include "occ_status.hpp"
#include "occ_sensor.hpp"
#include "utils.hpp"
namespace open_power
{
namespace occ
{

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value != this->occActive())
    {
        if (value)
        {
            // Bind the device
            device.bind();

            // Call into Manager to let know that we have bound
            // TODO: openbmc/openbmc#2285
           /* if (this->callBack)
            {
                this->callBack(value);
            }*/
        }
        else
        {
            // Call into Manager to let know that we will unbind.
            // Need to do this before doing un-bind since it will
            // result in slave error if Master is un-bound
            /*if (this->callBack)
            {
                this->callBack(value);
            }*/

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
        // TODO: openbmc/openbmc#2285
        // addErrorWatch();
    }
    return Base::Status::occActive(value);
}

// Callback handler when a device error is reported.
void Status::deviceErrorHandler()
{
    // This would deem OCC inactive
    this->occActive(false);

    // Reset the OCC
    this->resetOCC();
}

// Sends message to host control command handler to reset OCC
void Status::resetOCC()
{
    using namespace phosphor::logging;
    constexpr auto CONTROL_HOST_PATH = "/org/open_power/control/host0";
    constexpr auto CONTROL_HOST_INTF = "org.open_power.Control.Host";

    // This will throw exception on failure
    auto service = getService(bus, CONTROL_HOST_PATH, CONTROL_HOST_INTF);

    auto method = bus.new_method_call(service.c_str(),
                                      CONTROL_HOST_PATH,
                                      CONTROL_HOST_INTF,
                                      "Execute");
    // OCC Reset control command
    method.append(convertForMessage(
                Control::Host::Command::OCCReset).c_str());

    // OCC Sensor ID for callout reasons
    method.append(sdbusplus::message::variant<uint8_t>(
                        sensorMap.at(instance)));
    bus.call_noreply(method);
    return;
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
                      entry("COMMAND=%s",cmdCompleted.c_str()),
                      entry("STATUS=%s",cmdStatus.c_str()));

    if(Control::Host::convertResultFromString(cmdStatus) !=
            Control::Host::Result::Success)
    {
        if(Control::Host::convertCommandFromString(cmdCompleted) ==
                Control::Host::Command::OCCReset)
        {
            // Must be a Timeout. Log an Erorr trace
            log<level::ERR>("Error resetting the OCC.",
                    entry("PATH=%s", path.c_str()),
                    entry("SENSORID=0x%X",sensorMap.at(instance)));
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
