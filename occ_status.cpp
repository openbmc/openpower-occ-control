#include "occ_status.hpp"

#include "occ_manager.hpp"
#include "occ_sensor.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <fmt/core.h>

#ifdef POWER10
#include <com/ibm/Host/Target/server.hpp>
#endif
#include <phosphor-logging/log.hpp>

#include <filesystem>

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
            // Set the device active
            device.setActive(true);

            // Start watching for errors
            addErrorWatch();

            // Reset last OCC state
            lastState = 0;

            if (device.master())
            {
                // Update powercap bounds from OCC
                manager.updatePcapBounds();
            }

            // Update the OCC active sensor before notifying Manager
            Base::Status::occActive(value);

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
        addErrorWatch();
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
void Status::deviceError()
{
#ifdef POWER10
    if (pmode && device.master())
    {
        // Prevent mode changes
        pmode->setMasterActive(false);
    }
#endif

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

// Called from Manager::pollerTimerExpired() in preperation to POLL OCC.
void Status::readOccState()
{
    currentOccReadRetriesCount = occReadRetries;
    occReadStateNow();
}

#ifdef POWER10
// Special processing that needs to happen once the OCCs change to ACTIVE state
void Status::occsWentActive()
{
    CmdStatus status = CmdStatus::SUCCESS;

    status = pmode->sendModeChange();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "Status::occsWentActive: OCC mode change failed with status {}",
                status)
                .c_str());

        // Disable and reset to try recovering
        deviceError();
    }

    status = pmode->sendIpsData();
    if (status != CmdStatus::SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "Status::occsWentActive: Sending Idle Power Save Config data failed with status {}",
                status)
                .c_str());

        if (status == CmdStatus::COMM_FAILURE)
        {
            // Disable and reset to try recovering
            deviceError();
        }
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
            fmt::format("sendAmbient: valid: {}, Ambient: {}C, altitude: {}m",
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
    log<level::DEBUG>(fmt::format("sendAmbient: SEND_AMBIENT "
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
                    fmt::format(
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
                fmt::format(
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
            fmt::format(
                "sendAmbient: SEND_AMBIENT FAILED! with status 0x{:02X}",
                status)
                .c_str());

        if (status == CmdStatus::COMM_FAILURE)
        {
            // Disable and reset to try recovering
            deviceError();
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
            fmt::format(
                "safeStateDelayExpired: OCC{} is in SAFE state, requesting reset",
                instance)
                .c_str());
        // Disable and reset to try recovering
        deviceError();
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
            log<level::ERR>(
                fmt::format("Status::getHwmonPath(): path no longer exists: {}",
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
                        fmt::format(
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
                    fmt::format(
                        "Status::getHwmonPath(): error accessing {}: {}",
                        prefixPath.c_str(), e.what())
                        .c_str());
                tracedFail[instance] = true;
            }
        }
    }

    return hwmonPath;
}

// Called to read state and upon failure to read after occReadStateFailTimer.
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

        if (state != lastState)
        {
            // Trace OCC state changes
            log<level::INFO>(
                fmt::format("Status::readOccState: OCC{} state 0x{:02X}",
                            instance, state)
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
                        fmt::format(
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
            // Else not Valid state We would be in SAFE mode.
            // This captures both SAFE mode, and 0x00, or other invalid
            // state values.
            else
            {
                if (!safeStateDelayTimer.isEnabled())
                {
                    // start safe delay timer (before requesting reset)
                    using namespace std::literals::chrono_literals;
                    safeStateDelayTimer.restartOnce(60s);
                }
                // Not valid state, update sensors to Nan & not functional.
                stateValid = false;
            }
#else
            // Before P10 state not checked, only used good file open.
            stateValid = true;
#endif
        }
    }
    file.close();

    // if failed to Read a state or not a valid state -> Attempt retry
    // after 1 Second delay if allowed.
    if ((!goodFile) || (!stateValid))
    {
        if (!goodFile)
        {
            // If not able to read, OCC may be offline
            log<level::ERR>(
                fmt::format("Status::readOccState: open failed (errno={})",
                            openErrno)
                    .c_str());
        }
        else
        {
            // else this failed due to state not valid.
            log<level::ERR>(
                fmt::format(
                    "Status::readOccState: OCC{} Invalid state 0x{:02X}",
                    instance, state)
                    .c_str());
        }

#ifdef READ_OCC_SENSORS
        manager.setSensorValueToNonFunctional(instance);
#endif

        // See occReadRetries for number of retry attempts.
        if (currentOccReadRetriesCount > 0)
        {
            --currentOccReadRetriesCount;
#ifdef POWER10
            using namespace std::chrono_literals;
            occReadStateFailTimer.restartOnce(1s);
#endif
        }
        else
        {
#ifdef POWER10
            if (!stateValid && occActive())
            {
                if (!safeStateDelayTimer.isEnabled())
                {
                    log<level::ERR>(
                        "Starting 60 sec delay timer before requesting a reset");
                    // start safe delay timer (before requesting reset)
                    using namespace std::literals::chrono_literals;
                    safeStateDelayTimer.restartOnce(60s);
                }
            }
#else
            // State could not be determined, set it to NO State.
            lastState = 0;

            // Disable the ability to send Failed actions until OCC is
            // Active again.
            stateValid = false;

            // Disable and reset to try recovering
            deviceError();
#endif
        }
    }
}

} // namespace occ
} // namespace open_power
