#include "occ_status.hpp"

#include "occ_manager.hpp"
#include "occ_sensor.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <phosphor-logging/log.hpp>

#include <filesystem>
#include <format>

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
        log<level::INFO>(std::format("Status::occActive OCC{} changed to {}",
                                     instance, value)
                             .c_str());
        if (value)
        {
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
                log<level::ERR>(
                    std::format(
                        "Status::occActive: Unable to add error watch(s) for OCC{} watch: {}",
                        instance, e.what())
                        .c_str());
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
#ifdef POWER10
            if (pmode && device.master())
            {
                // Prevent mode changes
                pmode->setMasterActive(false);
            }
            if (safeStateDelayTimer.isEnabled())
            {
                // stop safe delay timer
                safeStateDelayTimer.setEnabled(false);
            }
#endif
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
            log<level::ERR>(
                std::format(
                    "Status::occActive: Unable to add error watch(s) again for OCC{} watch: {}",
                    instance, e.what())
                    .c_str());
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
#ifdef POWER10
    if (pmode && device.master())
    {
        // Prevent mode changes
        pmode->setMasterActive(false);
    }
#endif

    if (d.log)
    {
        FFDC::createOCCResetPEL(instance, d.path, d.err, d.callout);
    }

    // This would deem OCC inactive
    this->occActive(false);

    // Reset the OCC
    this->resetOCC();
}

// Sends message to host control command handler to reset OCC
void Status::resetOCC()
{
    log<level::INFO>(
        std::format(">>Status::resetOCC() - requesting reset for OCC{}",
                    instance)
            .c_str());
    this->occActive(false);
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
void Status::hostControlEvent(sdbusplus::message_t& msg)
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

#ifdef POWER10
// Special processing that needs to happen once the OCCs change to ACTIVE state
void Status::occsWentActive()
{
    CmdStatus status = CmdStatus::SUCCESS;

    // IPS data will get sent automatically after a mode change if the mode
    // supports it.
    pmode->needToSendIPS();

    status = pmode->sendModeChange();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            std::format(
                "Status::occsWentActive: OCC mode change failed with status {}",
                status)
                .c_str());

        // Disable and reset to try recovering
        deviceError();
    }
}

// Send Ambient and Altitude to the OCC
CmdStatus Status::sendAmbient(const uint8_t inTemp, const uint16_t inAltitude)
{
    CmdStatus status = CmdStatus::FAILURE;
    bool ambientValid = true;
    uint8_t ambientTemp = inTemp;
    uint16_t altitude = inAltitude;

    if (ambientTemp == 0xFF)
    {
        // Get latest readings from manager
        manager.getAmbientData(ambientValid, ambientTemp, altitude);
        log<level::DEBUG>(
            std::format("sendAmbient: valid: {}, Ambient: {}C, altitude: {}m",
                        ambientValid, ambientTemp, altitude)
                .c_str());
    }

    std::vector<std::uint8_t> cmd, rsp;
    cmd.reserve(11);
    cmd.push_back(uint8_t(CmdType::SEND_AMBIENT));
    cmd.push_back(0x00);                    // Data Length (2 bytes)
    cmd.push_back(0x08);                    //
    cmd.push_back(0x00);                    // Version
    cmd.push_back(ambientValid ? 0 : 0xFF); // Ambient Status
    cmd.push_back(ambientTemp);             // Ambient Temperature
    cmd.push_back(altitude >> 8);           // Altitude in meters (2 bytes)
    cmd.push_back(altitude & 0xFF);         //
    cmd.push_back(0x00);                    // Reserved (3 bytes)
    cmd.push_back(0x00);
    cmd.push_back(0x00);
    log<level::DEBUG>(std::format("sendAmbient: SEND_AMBIENT "
                                  "command to OCC{} ({} bytes)",
                                  instance, cmd.size())
                          .c_str());
    status = occCmd.send(cmd, rsp);
    if (status == CmdStatus::SUCCESS)
    {
        if (rsp.size() == 5)
        {
            if (RspStatus::SUCCESS != RspStatus(rsp[2]))
            {
                log<level::ERR>(
                    std::format(
                        "sendAmbient: SEND_AMBIENT failed with rspStatus 0x{:02X}",
                        rsp[2])
                        .c_str());
                dump_hex(rsp);
                status = CmdStatus::FAILURE;
            }
        }
        else
        {
            log<level::ERR>(
                std::format(
                    "sendAmbient: INVALID SEND_AMBIENT response length:{}",
                    rsp.size())
                    .c_str());
            dump_hex(rsp);
            status = CmdStatus::FAILURE;
        }
    }
    else
    {
        log<level::ERR>(
            std::format(
                "sendAmbient: SEND_AMBIENT FAILED! with status 0x{:02X}",
                status)
                .c_str());

        if (status == CmdStatus::COMM_FAILURE)
        {
            // Disable due to OCC comm failure and reset to try recovering
            deviceError(Error::Descriptor(OCC_COMM_ERROR_PATH));
        }
    }

    return status;
}

// Called when safe timer expires to determine if OCCs need to be reset
void Status::safeStateDelayExpired()
{
    if (this->occActive())
    {
        log<level::INFO>(
            std::format(
                "safeStateDelayExpired: OCC{} state missing or not valid, requesting reset",
                instance)
                .c_str());
        // Disable and reset to try recovering
        deviceError(Error::Descriptor(SAFE_ERROR_PATH));
    }
}
#endif // POWER10

fs::path Status::getHwmonPath()
{
    using namespace std::literals::string_literals;

    if (!fs::exists(hwmonPath))
    {
        static bool tracedFail[8] = {0};

        if (!hwmonPath.empty())
        {
            log<level::WARNING>(
                std::format("Status::getHwmonPath(): path no longer exists: {}",
                            hwmonPath.c_str())
                    .c_str());
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
                    log<level::ERR>(
                        std::format(
                            "Status::getHwmonPath(): Found multiple ({}) hwmon paths!",
                            numDirs)
                            .c_str());
                    tracedFail[instance] = true;
                }
            }
        }
        catch (const fs::filesystem_error& e)
        {
            if (!tracedFail[instance])
            {
                log<level::ERR>(
                    std::format(
                        "Status::getHwmonPath(): error accessing {}: {}",
                        prefixPath.c_str(), e.what())
                        .c_str());
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
    bool goodFile = false;

    // open file.
    file.open(filename, std::ios::in);
    const int openErrno = errno;

    // File is open and state can be used.
    if (file.is_open() && file.good())
    {
        goodFile = true;
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
                log<level::ERR>(
                    std::format(
                        "readOccState: Failed to read OCC{} state: Read error on I/O operation -{}",
                        instance, errorBits)
                        .c_str());
                lastOccReadStatus = -1;
            }
            goodFile = false;
        }

        if (goodFile && (state != lastState))
        {
            // Trace OCC state changes
            log<level::INFO>(
                std::format(
                    "Status::readOccState: OCC{} state 0x{:02X} (lastState: 0x{:02X})",
                    instance, state, lastState)
                    .c_str());
            lastState = state;
#ifdef POWER10
            if (OccState(state) == OccState::ACTIVE)
            {
                if (pmode && device.master())
                {
                    // Set the master OCC on the PowerMode object
                    pmode->setMasterOcc(path);
                    // Enable mode changes
                    pmode->setMasterActive();

                    // Special processing by master OCC when it goes active
                    occsWentActive();
                }

                CmdStatus status = sendAmbient();
                if (status != CmdStatus::SUCCESS)
                {
                    log<level::ERR>(
                        std::format(
                            "readOccState: Sending Ambient failed with status {}",
                            status)
                            .c_str());
                }
            }

            // If OCC in known Good State.
            if ((OccState(state) == OccState::ACTIVE) ||
                (OccState(state) == OccState::CHARACTERIZATION) ||
                (OccState(state) == OccState::OBSERVATION))
            {
                // Good OCC State then sensors valid again
                stateValid = true;

                if (safeStateDelayTimer.isEnabled())
                {
                    // stop safe delay timer (no longer in SAFE state)
                    safeStateDelayTimer.setEnabled(false);
                }
            }
            else
            {
                // OCC is in SAFE or some other unsupported state
                if (!safeStateDelayTimer.isEnabled())
                {
                    log<level::ERR>(
                        std::format(
                            "readOccState: Invalid OCC{} state of {}, starting safe state delay timer",
                            instance, state)
                            .c_str());
                    // start safe delay timer (before requesting reset)
                    using namespace std::literals::chrono_literals;
                    safeStateDelayTimer.restartOnce(60s);
                }
                // Not a supported state (update sensors to NaN and not
                // functional)
                stateValid = false;
            }
#else
            // Before P10 state not checked, only used good file open.
            stateValid = true;
#endif
        }
    }
#ifdef POWER10
    else
    {
        // Unable to read state
        stateValid = false;
    }
#endif
    file.close();

    // if failed to Read a state or not a valid state -> Attempt retry
    // after 1 Second delay if allowed.
    if ((!goodFile) || (!stateValid))
    {
        if (!goodFile)
        {
            // If not able to read, OCC may be offline
            if (openErrno != lastOccReadStatus)
            {
                log<level::ERR>(
                    std::format(
                        "Status::readOccState: open/read failed trying to read OCC{} state (open errno={})",
                        instance, openErrno)
                        .c_str());
                lastOccReadStatus = openErrno;
            }
        }
        else
        {
            // else this failed due to state not valid.
            if (state != lastState)
            {
                log<level::ERR>(
                    std::format(
                        "Status::readOccState: OCC{} Invalid state 0x{:02X} (last state: 0x{:02X})",
                        instance, state, lastState)
                        .c_str());
            }
        }

#ifdef READ_OCC_SENSORS
        manager.setSensorValueToNaN(instance);
#endif

        // See occReadRetries for number of retry attempts.
        if (currentOccReadRetriesCount > 0)
        {
            --currentOccReadRetriesCount;
        }
        else
        {
            log<level::ERR>(
                std::format("readOccState: failed to read OCC{} state!",
                            instance)
                    .c_str());

            // State could not be determined, set it to NO State.
            lastState = 0;

            // Disable the ability to send Failed actions until OCC is
            // Active again.
            stateValid = false;

            // Disable due to OCC comm failure and reset to try recovering
            deviceError(Error::Descriptor(OCC_COMM_ERROR_PATH));

            // Reset retry count (for next attempt after recovery)
            currentOccReadRetriesCount = occReadRetries;
        }
    }
    else
    {
        if (lastOccReadStatus != 0)
        {
            log<level::INFO>(
                std::format(
                    "Status::readOccState: successfully read OCC{} state: {}",
                    instance, state)
                    .c_str());
            lastOccReadStatus = 0; // no error
        }
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
            log<level::DEBUG>(
                std::format(
                    "updateThrottle: OCC{} no longer throttled (prior reason: {})",
                    instance, throttleCause)
                    .c_str());
            throttleCause = THROTTLED_NONE;
            throttleHandle->throttled(false);
            throttleHandle->throttleCauses({});
        }
        else
        {
            log<level::DEBUG>(
                std::format(
                    "updateThrottle: OCC{} is throttled with reason {} (prior reason: {})",
                    instance, newThrottleCause, throttleCause)
                    .c_str());
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
    log<level::DEBUG>(
        std::format("readProcAssociation: getting endpoints for {} ({})",
                    managingPath, path)
            .c_str());
    try
    {
        utils::PropertyValue procPathProperty{};
        procPathProperty = utils::getProperty(
            managingPath, "xyz.openbmc_project.Association", "endpoints");
        auto result = std::get<std::vector<std::string>>(procPathProperty);
        if (result.size() > 0)
        {
            procPath = result[0];
            log<level::INFO>(
                std::format("readProcAssociation: OCC{} has proc={}", instance,
                            procPath.c_str())
                    .c_str());
        }
        else
        {
            log<level::ERR>(
                std::format(
                    "readProcAssociation: No processor associated with OCC{} / {}",
                    instance, path)
                    .c_str());
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format(
                "readProcAssociation: Unable to get proc assocated with {} - {}",
                path, e.what())
                .c_str());
        procPath = {};
    }
}

} // namespace occ
} // namespace open_power
