#include "legacy/occ_status_legacy.hpp"

#include "legacy/occ_manager_legacy.hpp"
#include "occ_sensor.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <phosphor-logging/lg2.hpp>
#include <filesystem>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

using ThrottleObj =
    sdbusplus::xyz::openbmc_project::Control::Power::server::Throttle;

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value != this->occActive())
    {
        lg2::info("Status::occActive OCC{INST} changed to {STATE}", "INST",
                  instance, "STATE", value);
        if (value)
        {
            // OCC is active
            // Clear prior throttle reason (before setting device active)
            updateThrottle(false, THROTTLED_ALL);

            // Set the device active
            device.setActive(true);

            // Reset last OCC state
            lastState = 0;

            // Start watching for errors (throttles, etc)
            try
            {
                addErrorWatch();
            }
            catch (const OpenFailure& e)
            {
                // Failed to add watch for throttle events, request reset to try
                // to recover comm
                lg2::error(
                    "Status::occActive: Unable to add error watch(s) for OCC{INST} watch: {ERROR}",
                    "INST", instance, "ERROR", e.what());
                deviceError(Error::Descriptor(OCC_COMM_ERROR_PATH));
                return Base::Status::occActive(false);
            }

            // Update the OCC active sensor
            Base::Status::occActive(value);

            if (device.master())
            {
                // Update powercap bounds from OCC
                manager.updatePcapBounds();
            }

            // Call into Manager to let know that we have bound
            if (this->managerCallBack)
            {
                this->managerCallBack(instance, value);
            }
        }
        else
        {
            // OCC is no longer active
            // Call into Manager to let know that we will unbind.
            if (this->managerCallBack)
            {
                this->managerCallBack(instance, value);
            }

            // Stop watching for errors
            removeErrorWatch();

            // Set the device inactive
            device.setActive(false);

            // Clear throttles (OCC not active after disabling device)
            updateThrottle(false, THROTTLED_ALL);
        }
    }
    else if (value && !device.active())
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
        device.setActive(true);

        // Add error watch again
        try
        {
            addErrorWatch();
        }
        catch (const OpenFailure& e)
        {
            // Failed to add watch for throttle events, request reset to try to
            // recover comm
            lg2::error(
                "Status::occActive: Unable to add error watch(s) again for OCC{INST} watch: {ERROR}",
                "INST", instance, "ERROR", e.what());
            deviceError(Error::Descriptor(OCC_COMM_ERROR_PATH));
            return Base::Status::occActive(false);
        }
    }
    else if (!value && device.active())
    {
        removeErrorWatch();

        // In the event that the application never receives the active signal
        // even though the OCC is active (this can occur if the BMC is rebooted
        // with the host on, since the initial OCC driver probe will discover
        // the OCCs), this application needs to be able to unbind the device
        // when we get the OCC inactive signal.
        device.setActive(false);
    }
    return Base::Status::occActive(value);
}

// Callback handler when a device error is reported.
void Status::deviceError(Error::Descriptor d)
{
    if (d.log)
    {
        FFDC::createOCCResetPEL(instance, d.path, d.err, d.callout,
                                d.isInventoryCallout);
    }

    // This would deem OCC inactive
    this->occActive(false);

    // Reset the OCC
    this->resetOCC();
}

// Sends message to host control command handler to reset OCC
void Status::resetOCC()
{
    lg2::info(">>Status::resetOCC() - requesting reset for OCC{INST}", "INST",
              instance);
    this->occActive(false);
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
}

// Handler called by Host control command handler to convey the
// status of the executed command
void Status::hostControlEvent(sdbusplus::message_t& msg)
{
    std::string cmdCompleted{};
    std::string cmdStatus{};

    msg.read(cmdCompleted, cmdStatus);

    lg2::debug("Host control signal values: command={CMD}, status={STATUS}",
               "CMD", cmdCompleted, "STATUS", cmdStatus);

    if (Control::Host::convertResultFromString(cmdStatus) !=
        Control::Host::Result::Success)
    {
        if (Control::Host::convertCommandFromString(cmdCompleted) ==
            Control::Host::Command::OCCReset)
        {
            // Must be a Timeout. Log an Error trace
            lg2::error(
                "Error resetting the OCC: path={PATH}, sensorid={SENSOR}",
                "PATH", path, "SENSOR", std::get<0>(sensorMap.at(instance)));
        }
    }
    return;
}

// Called from Manager::pollerTimerExpired() in preperation to POLL OCC.
void Status::readOccState()
{
    if (stateValid)
    {
        // Reset retry count (since state is good)
        currentOccReadRetriesCount = occReadRetries;
    }
    occReadStateNow();
}

fs::path Status::getHwmonPath()
{
    using namespace std::literals::string_literals;

    if (!fs::exists(hwmonPath))
    {
        static bool tracedFail[8] = {0};

        if (!hwmonPath.empty())
        {
            lg2::warning(
                "Status::getHwmonPath(): path no longer exists: {PATH}", "PATH",
                hwmonPath);
            hwmonPath.clear();
        }

        // Build the base HWMON path
        fs::path prefixPath =
            fs::path{OCC_HWMON_PATH + "occ-hwmon."s +
                     std::to_string(instance + 1) + "/hwmon/"s};

        // Get the hwmonXX directory name
        try
        {
            // there should only be one directory
            const int numDirs = std::distance(
                fs::directory_iterator(prefixPath), fs::directory_iterator{});
            if (numDirs == 1)
            {
                hwmonPath = *fs::directory_iterator(prefixPath);
                tracedFail[instance] = false;
            }
            else
            {
                if (!tracedFail[instance])
                {
                    lg2::error(
                        "Status::getHwmonPath(): Found multiple ({NUM}) hwmon paths!",
                        "NUM", numDirs);
                    tracedFail[instance] = true;
                }
            }
        }
        catch (const fs::filesystem_error& e)
        {
            if (!tracedFail[instance])
            {
                lg2::error(
                    "Status::getHwmonPath(): error accessing {PATH}: {ERROR}",
                    "PATH", prefixPath, "ERROR", e.what());
                tracedFail[instance] = true;
            }
        }
    }

    return hwmonPath;
}

// Called to read state and handle any errors
void Status::occReadStateNow()
{
    unsigned int state;
    const fs::path filename =
        fs::path(DEV_PATH) /
        fs::path(sysfsName + "." + std::to_string(instance + 1)) / "occ_state";

    std::ifstream file;
    bool stateWasRead = false;

    // open file.
    file.open(filename, std::ios::in);
    const int openErrno = errno;

    // File is open and state can be used.
    if (file.is_open() && file.good())
    {
        stateWasRead = true;
        file >> state;
        // Read the error code (if any) to check status of the read
        std::ios_base::iostate readState = file.rdstate();
        if (readState)
        {
            // There was a failure reading the file
            if (lastOccReadStatus != -1)
            {
                // Trace error bits
                std::string errorBits = "";
                if (readState & std::ios_base::eofbit)
                {
                    errorBits += " EOF";
                }
                if (readState & std::ios_base::failbit)
                {
                    errorBits += " failbit";
                }
                if (readState & std::ios_base::badbit)
                {
                    errorBits += " badbit";
                }
                lg2::error(
                    "readOccState: Failed to read OCC{INST} state: Read error on I/O operation - {ERROR}",
                    "INST", instance, "ERROR", errorBits);
                lastOccReadStatus = -1;
            }
            stateWasRead = false;
        }

        if (stateWasRead && (state != lastState))
        {
            // Trace OCC state changes
            lg2::info(
                "Status::readOccState: OCC{INST} state {STATE} (lastState: {PRIOR})",
                "INST", instance, "STATE", lg2::hex, state, "PRIOR", lg2::hex,
                lastState);
            lastState = state;
            // Before P10 state not checked, only used good file open.
            stateValid = true;
        }
    }

    file.close();

    // if failed to read the OCC state -> Attempt retry
    if (!stateWasRead)
    {
        // If not able to read, OCC may be offline
        if (openErrno != lastOccReadStatus)
        {
            lg2::error(
                "Status::readOccState: open/read failed trying to read OCC{INST} state (open errno={ERROR})",
                "INST", instance, "ERROR", openErrno);
            lastOccReadStatus = openErrno;
        }

        // See occReadRetries for number of retry attempts.
        if (currentOccReadRetriesCount > 0)
        {
            --currentOccReadRetriesCount;
        }
        else
        {
            lg2::error(
                "readOccState: failed to read OCC{INST} state! (last state: {PRIOR})",
                "INST", instance, "PRIOR", lg2::hex, lastState);

            // State could not be determined, set it to NO State.
            lastState = 0;

            // Disable the ability to send Failed actions until OCC is
            // Active again.
            stateValid = false;

            // Disable due to OCC comm failure and reset to try recovering
            // (processor callout will be added)
            deviceError(Error::Descriptor(OCC_COMM_ERROR_PATH, ECOMM,
                                          procPath.c_str(), true));

            // Reset retry count (for next attempt after recovery)
            currentOccReadRetriesCount = occReadRetries;
        }
    }
    else if (lastOccReadStatus != 0)
    {
        lg2::info("readOccState: successfully read OCC{INST} state: {STATE}",
                  "INST", instance, "STATE", state);
        lastOccReadStatus = 0; // no error
    }
}

// Update processor throttle status on dbus
void Status::updateThrottle(const bool isThrottled, const uint8_t newReason)
{
    if (!throttleHandle)
    {
        return;
    }

    uint8_t newThrottleCause = throttleCause;

    if (isThrottled) // throttled due to newReason
    {
        if ((newReason & throttleCause) == 0)
        {
            // set the bit(s) for passed in reason
            newThrottleCause |= newReason;
        }
        // else no change
    }
    else // no longer throttled due to newReason
    {
        if ((newReason & throttleCause) != 0)
        {
            // clear the bit(s) for passed in reason
            newThrottleCause &= ~newReason;
        }
        // else no change
    }

    if (newThrottleCause != throttleCause)
    {
        if (newThrottleCause == THROTTLED_NONE)
        {
            lg2::debug(
                "updateThrottle: OCC{INST} no longer throttled (prior reason: {REASON})",
                "INST", instance, "REASON", throttleCause);
            throttleCause = THROTTLED_NONE;
            throttleHandle->throttled(false);
            throttleHandle->throttleCauses({});
        }
        else
        {
            lg2::debug(
                "updateThrottle: OCC{INST} is throttled with reason {REASON} (prior reason: {PRIOR})",
                "INST", instance, "REASON", newThrottleCause, "PRIOR",
                throttleCause);
            throttleCause = newThrottleCause;

            std::vector<ThrottleObj::ThrottleReasons> updatedCauses;
            if (throttleCause & THROTTLED_POWER)
            {
                updatedCauses.push_back(
                    throttleHandle->ThrottleReasons::PowerLimit);
            }
            if (throttleCause & THROTTLED_THERMAL)
            {
                updatedCauses.push_back(
                    throttleHandle->ThrottleReasons::ThermalLimit);
            }
            if (throttleCause & THROTTLED_SAFE)
            {
                updatedCauses.push_back(
                    throttleHandle->ThrottleReasons::ManagementDetectedFault);
            }
            throttleHandle->throttleCauses(updatedCauses);
            throttleHandle->throttled(true);
        }
    }
    // else no change to throttle status
}

// Get processor path associated with this OCC
void Status::readProcAssociation()
{
    std::string managingPath = path + "/power_managing";
    lg2::debug("readProcAssociation: getting endpoints for {MANAGE} ({PATH})",
               "MANAGE", managingPath, "PATH", path);
    try
    {
        utils::PropertyValue procPathProperty{};
        procPathProperty = utils::getProperty(
            managingPath, "xyz.openbmc_project.Association", "endpoints");
        auto result = std::get<std::vector<std::string>>(procPathProperty);
        if (result.size() > 0)
        {
            procPath = result[0];
            lg2::info("readProcAssociation: OCC{INST} has proc={PATH}", "INST",
                      instance, "PATH", procPath);
        }
        else
        {
            lg2::error(
                "readProcAssociation: No processor associated with OCC{INST} / {PATH}",
                "INST", instance, "PATH", path);
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "readProcAssociation: Unable to get proc assocated with {PATH} - {ERROR}",
            "PATH", path, "ERROR", e.what());
        procPath = {};
    }
}

} // namespace occ
} // namespace open_power
