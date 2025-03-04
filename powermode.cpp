#include "powermode.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef POWERVM_CHECK
#include <com/ibm/Host/Target/server.hpp>
#endif
#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Control/Power/Mode/server.hpp>

#include <cassert>
#include <fstream>
#include <regex>

namespace open_power
{
namespace occ
{
namespace powermode
{

using namespace phosphor::logging;
using namespace std::literals::string_literals;
using namespace sdbusplus::org::open_power::OCC::Device::Error;

using Mode = sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;

using NotAllowed = sdbusplus::xyz::openbmc_project::Common::Error::NotAllowed;

// List of all Power Modes that are currently supported (and in Redfish)
#define VALID_POWER_MODE_SETTING(mode)                                         \
    ((mode == SysPwrMode::STATIC) || (mode == SysPwrMode::POWER_SAVING) ||     \
     (mode == SysPwrMode::BALANCED_PERF) || (mode == SysPwrMode::MAX_PERF) ||  \
     (mode == SysPwrMode::EFF_FAVOR_POWER) ||                                  \
     (mode == SysPwrMode::EFF_FAVOR_PERF))
// List of OEM Power Modes that are currently supported
#define VALID_OEM_POWER_MODE_SETTING(mode)                                     \
    ((mode == SysPwrMode::SFP) || (mode == SysPwrMode::FFO) ||                 \
     (mode == SysPwrMode::MAX_FREQ) ||                                         \
     (mode == SysPwrMode::NON_DETERMINISTIC))
// List of all Power Modes that disable IPS
#define IS_ECO_MODE(mode)                                                      \
    ((mode == SysPwrMode::EFF_FAVOR_POWER) ||                                  \
     (mode == SysPwrMode::EFF_FAVOR_PERF))

// Constructor
PowerMode::PowerMode(const Manager& managerRef, const char* modePath,
                     const char* ipsPath, EventPtr& event) :
    ModeInterface(utils::getBus(), modePath,
                  ModeInterface::action::emit_no_signals),
    manager(managerRef),
    ipsMatch(utils::getBus(),
             sdbusplus::bus::match::rules::propertiesChanged(PIPS_PATH,
                                                             PIPS_INTERFACE),
             [this](auto& msg) { this->ipsChanged(msg); }),
    defaultsUpdateMatch(
        utils::getBus(),
        sdbusplus::bus::match::rules::propertiesChangedNamespace(
            "/xyz/openbmc_project/inventory", PMODE_DEFAULT_INTERFACE),
        [this](auto& msg) { this->defaultsReady(msg); }),
    masterOccSet(false), masterActive(false), ipsObjectPath(ipsPath),
    event(event)
{
    // Get supported power modes from entity manager
    if (false == getSupportedModes())
    {
        // Did not find them so use default customer modes
        using Mode =
            sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;
        // Update power modes that will be allowed by the Redfish interface
        ModeInterface::allowedPowerModes(
            {Mode::PowerMode::Static, Mode::PowerMode::MaximumPerformance,
             Mode::PowerMode::PowerSaving});
    }

    SysPwrMode currentMode;
    uint16_t oemModeData = 0;
    // Read the persisted power mode
    if (getMode(currentMode, oemModeData))
    {
        // Validate persisted mode is supported
        if (isValidMode(currentMode))
        {
            // Update power mode on DBus and create IPS object if allowed
            updateDbusMode(currentMode);
        }
        else
        {
            lg2::error("PowerMode: Persisted power mode ({MODE}/{DATA}) is not "
                       "valid. Reading system default mode",
                       "MODE", currentMode, "DATA", oemModeData);
            persistedData.invalidateMode();
            // Read default power mode
            initPersistentData();
        }
    }
};

void PowerMode::createIpsObject()
{
    if (!ipsObject)
    {
        lg2::info("createIpsObject: Creating IPS object");
        ipsObject =
            std::make_unique<IpsInterface>(utils::getBus(), ipsObjectPath);

        uint8_t enterUtil, exitUtil;
        uint16_t enterTime, exitTime;
        bool ipsEnabled;
        // Read the persisted Idle Power Saver parametres
        if (getIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime))
        {
            // Update Idle Power Saver parameters on DBus
            updateDbusIPS(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);
        }

        // Starts watching for IPS state changes.
        addIpsWatch(true);

        needToSendIpsData = true;
    }
}

void PowerMode::removeIpsObject()
{
    if (ipsObject)
    {
        // Stop watching for IPS state changes.
        removeIpsWatch();

        lg2::info("removeIpsObject: Deleting IPS object");
        ipsObject.reset(nullptr);
    }
    needToSendIpsData = false;
}

// Set the Master OCC
void PowerMode::setMasterOcc(const std::string& masterOccPath)
{
    if (masterOccSet)
    {
        if (masterOccPath != path)
        {
            lg2::error(
                "PowerMode::setMasterOcc: Master changed (was OCC{INST}, {PATH})",
                "INST", occInstance, "PATH", masterOccPath);
            if (occCmd)
            {
                occCmd.reset();
            }
        }
    }
    path = masterOccPath;
    occInstance = path.back() - '0';
    lg2::debug("PowerMode::setMasterOcc(OCC{INST}, {PATH})", "INST",
               occInstance, "PATH", path);
    if (!occCmd)
    {
        occCmd = std::make_unique<open_power::occ::OccCommand>(
            occInstance, path.c_str());
    }
    masterOccSet = true;
};

// Set the state of power mode lock. Writing persistent data via dbus method.
bool PowerMode::powerModeLock()
{
    lg2::info("PowerMode::powerModeLock: locking mode change");
    persistedData.updateModeLock(true); // write persistent data
    return true;
}

// Get the state of power mode. Reading persistent data via dbus method.
bool PowerMode::powerModeLockStatus()
{
    bool status = persistedData.getModeLock(); // read persistent data
    lg2::info("PowerMode::powerModeLockStatus: {STATUS}", "STATUS",
              status ? "locked" : "unlocked");
    return status;
}

// Called from OCC PassThrough interface (via CE login / BMC command line)
bool PowerMode::setMode(const SysPwrMode newMode, const uint16_t oemModeData)
{
    if (persistedData.getModeLock())
    {
        lg2::info("PowerMode::setMode: mode change blocked");
        return false;
    }

    if (updateDbusMode(newMode) == false)
    {
        // Unsupported mode
        return false;
    }

    // Save mode
    persistedData.updateMode(newMode, oemModeData);

    // Send mode change to OCC
    if (sendModeChange() != CmdStatus::SUCCESS)
    {
        // Mode change failed
        return false;
    }

    return true;
}

// Convert PowerMode value to occ-control internal SysPwrMode
// Returns SysPwrMode::NO_CHANGE if mode not valid
SysPwrMode getInternalMode(const Mode::PowerMode& mode)
{
    if (mode == Mode::PowerMode::MaximumPerformance)
    {
        return SysPwrMode::MAX_PERF;
    }
    else if (mode == Mode::PowerMode::PowerSaving)
    {
        return SysPwrMode::POWER_SAVING;
    }
    else if (mode == Mode::PowerMode::Static)
    {
        return SysPwrMode::STATIC;
    }
    else if (mode == Mode::PowerMode::EfficiencyFavorPower)
    {
        return SysPwrMode::EFF_FAVOR_POWER;
    }
    else if (mode == Mode::PowerMode::EfficiencyFavorPerformance)
    {
        return SysPwrMode::EFF_FAVOR_PERF;
    }
    else if (mode == Mode::PowerMode::BalancedPerformance)
    {
        return SysPwrMode::BALANCED_PERF;
    }

    lg2::warning("getInternalMode: Invalid PowerMode specified");
    return SysPwrMode::NO_CHANGE;
}

// Convert PowerMode string to OCC SysPwrMode
// Returns NO_CHANGE if OEM or unsupported mode
SysPwrMode convertStringToMode(const std::string& i_modeString)
{
    SysPwrMode newMode = SysPwrMode::NO_CHANGE;
    try
    {
        Mode::PowerMode newPMode =
            Mode::convertPowerModeFromString(i_modeString);
        newMode = getInternalMode(newPMode);
    }
    catch (const std::exception& e)
    {
        // Strip off prefix to to search OEM modes not part of Redfish
        auto prefix = PMODE_INTERFACE + ".PowerMode."s;
        std::string shortMode = i_modeString;
        std::string::size_type index = i_modeString.find(prefix);
        if (index != std::string::npos)
        {
            shortMode.erase(0, prefix.length());
        }
        if (shortMode == "FFO")
        {
            newMode = SysPwrMode::FFO;
        }
        else if (shortMode == "SFP")
        {
            newMode = SysPwrMode::SFP;
        }
        else if (shortMode == "MaxFrequency")
        {
            newMode = SysPwrMode::MAX_FREQ;
        }
        else if (shortMode == "NonDeterministic")
        {
            newMode = SysPwrMode::NON_DETERMINISTIC;
        }
        else
        {
            lg2::error(
                "convertStringToMode: Invalid Power Mode: {MODE} ({DATA})",
                "MODE", shortMode, "DATA", e.what());
        }
    }
    return newMode;
}

// Check if Hypervisor target is PowerVM
bool isPowerVM()
{
    bool powerVmTarget = true;
#ifdef POWERVM_CHECK
    namespace Hyper = sdbusplus::com::ibm::Host::server;
    constexpr auto HYPE_PATH = "/com/ibm/host0/hypervisor";
    constexpr auto HYPE_INTERFACE = "com.ibm.Host.Target";
    constexpr auto HYPE_PROP = "Target";

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

    lg2::debug("isPowerVM returning {VAL}", "VAL", powerVmTarget);
#endif

    return powerVmTarget;
}

// Initialize persistent data and return true if successful
bool PowerMode::initPersistentData()
{
    if (!persistedData.modeAvailable())
    {
        // Read the default mode
        SysPwrMode currentMode;
        if (!getDefaultMode(currentMode))
        {
            // Unable to read defaults
            return false;
        }
        lg2::info("PowerMode::initPersistentData: Using default mode: {MODE}",
                  "MODE", currentMode);

        // Save default mode as current mode
        persistedData.updateMode(currentMode, 0);

        // Write default mode to DBus and create IPS object if allowed
        updateDbusMode(currentMode);
    }

    if (!persistedData.ipsAvailable())
    {
        // Read the default IPS parameters, write persistent file and update
        // DBus
        return useDefaultIPSParms();
    }
    return true;
}

// Get the requested power mode and return true if successful
bool PowerMode::getMode(SysPwrMode& currentMode, uint16_t& oemModeData)
{
    currentMode = SysPwrMode::NO_CHANGE;
    oemModeData = 0;

    if (!persistedData.getMode(currentMode, oemModeData))
    {
        // Persistent data not initialized, read defaults and update DBus
        if (!initPersistentData())
        {
            // Unable to read defaults from entity manager yet
            return false;
        }
        return persistedData.getMode(currentMode, oemModeData);
    }

    return true;
}

// Set the power mode on DBus and create IPS object if allowed/needed
bool PowerMode::updateDbusMode(const SysPwrMode newMode)
{
    if (!isValidMode(newMode))
    {
        lg2::error(
            "PowerMode::updateDbusMode - Requested power mode not supported: {MODE}",
            "MODE", newMode);
        return false;
    }

    ModeInterface::PowerMode dBusMode = Mode::PowerMode::OEM;
    if (customerModeList.contains(newMode))
    {
        // Convert mode for DBus
        switch (newMode)
        {
            case SysPwrMode::STATIC:
                dBusMode = Mode::PowerMode::Static;
                break;
            case SysPwrMode::POWER_SAVING:
                dBusMode = Mode::PowerMode::PowerSaving;
                break;
            case SysPwrMode::MAX_PERF:
                dBusMode = Mode::PowerMode::MaximumPerformance;
                break;
            case SysPwrMode::EFF_FAVOR_POWER:
                dBusMode = Mode::PowerMode::EfficiencyFavorPower;
                break;
            case SysPwrMode::EFF_FAVOR_PERF:
                dBusMode = Mode::PowerMode::EfficiencyFavorPerformance;
                break;
            case SysPwrMode::BALANCED_PERF:
                dBusMode = Mode::PowerMode::BalancedPerformance;
                break;
            default:
                break;
        }
    }
    // else return OEM mode

    // Determine if IPS is allowed and create/remove as needed
    if (IS_ECO_MODE(newMode))
    {
        removeIpsObject();
    }
    else
    {
        createIpsObject();
    }

    ModeInterface::powerMode(dBusMode);

    return true;
}

// Send mode change request to the master OCC
CmdStatus PowerMode::sendModeChange()
{
    CmdStatus status;

    SysPwrMode newMode;
    uint16_t oemModeData = 0;
    getMode(newMode, oemModeData);

    if (isValidMode(newMode))
    {
        if (IS_ECO_MODE(newMode))
        {
            // Customer no longer able to enable IPS
            removeIpsObject();
        }
        else
        {
            // Customer now able to enable IPS
            if (!ipsObject)
            {
                createIpsObject();
            }
            else
            {
                if (!watching)
                {
                    // Starts watching for IPS state changes.
                    addIpsWatch(true);
                }
            }
        }

        if (!masterActive || !masterOccSet)
        {
            // Nothing to do until OCC goes active
            lg2::debug("PowerMode::sendModeChange: OCC master not active");
            return CmdStatus::SUCCESS;
        }

        if (!isPowerVM())
        {
            // Mode change is only supported on PowerVM systems
            lg2::debug(
                "PowerMode::sendModeChange: MODE CHANGE does not get sent on non-PowerVM systems");
            return CmdStatus::SUCCESS;
        }

        std::vector<std::uint8_t> cmd, rsp;
        cmd.reserve(9);
        cmd.push_back(uint8_t(CmdType::SET_MODE_AND_STATE));
        cmd.push_back(0x00); // Data Length (2 bytes)
        cmd.push_back(0x06);
        cmd.push_back(0x30); // Data (Version)
        cmd.push_back(uint8_t(OccState::NO_CHANGE));
        cmd.push_back(uint8_t(newMode));
        cmd.push_back(oemModeData >> 8);   // Mode Data (Freq Point)
        cmd.push_back(oemModeData & 0xFF); //
        cmd.push_back(0x00);               // reserved
        lg2::info(
            "PowerMode::sendModeChange: SET_MODE({MODE},{DATA}) command to OCC{INST} ({LEN} bytes)",
            "MODE", uint8_t(newMode), "DATA", oemModeData, "INST", occInstance,
            "LEN", cmd.size());
        status = occCmd->send(cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() == 5)
            {
                if (RspStatus::SUCCESS == RspStatus(rsp[2]))
                {
                    if (needToSendIpsData)
                    {
                        // Successful mode change and IPS is now allowed, so
                        // send IPS config
                        sendIpsData();
                    }
                }
                else
                {
                    lg2::error(
                        "PowerMode::sendModeChange: SET MODE failed with status {STATUS}",
                        "STATUS", lg2::hex, rsp[2]);
                    dump_hex(rsp);
                    status = CmdStatus::FAILURE;
                }
            }
            else
            {
                lg2::error(
                    "PowerMode::sendModeChange: INVALID SET MODE response");
                dump_hex(rsp);
                status = CmdStatus::FAILURE;
            }
        }
        else
        {
            lg2::error(
                "PowerMode::sendModeChange: SET_MODE FAILED with status={STATUS}",
                "STATUS", lg2::hex, uint8_t(status));
        }
    }
    else
    {
        lg2::error(
            "PowerMode::sendModeChange: Unable to set power mode to {MODE}",
            "MODE", newMode);
        status = CmdStatus::FAILURE;
    }

    return status;
}

// Handle IPS changed event (from GUI/Redfish)
void PowerMode::ipsChanged(sdbusplus::message_t& msg)
{
    bool parmsChanged = false;
    std::string interface;
    std::map<std::string, std::variant<bool, uint8_t, uint64_t>>
        ipsProperties{};
    msg.read(interface, ipsProperties);

    // Read persisted values
    bool ipsEnabled;
    uint8_t enterUtil, exitUtil;
    uint16_t enterTime, exitTime;
    getIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);

    if (!ipsObject)
    {
        lg2::warning(
            "ipsChanged: Idle Power Saver can not be modified in an ECO power mode");
        return;
    }

    // Check for any changed data
    auto ipsEntry = ipsProperties.find(IPS_ENABLED_PROP);
    if (ipsEntry != ipsProperties.end())
    {
        ipsEnabled = std::get<bool>(ipsEntry->second);
        lg2::info("Idle Power Saver change: Enabled={STAT}", "STAT",
                  ipsEnabled);
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        enterUtil = std::get<uint8_t>(ipsEntry->second);
        lg2::info("Idle Power Saver change: Enter Util={UTIL}%", "UTIL",
                  enterUtil);
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        enterTime =
            std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        lg2::info("Idle Power Saver change: Enter Time={TIME}sec", "TIME",
                  enterTime);
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        exitUtil = std::get<uint8_t>(ipsEntry->second);
        lg2::info("Idle Power Saver change: Exit Util={UTIL}%", "UTIL",
                  exitUtil);
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        exitTime = std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        lg2::info("Idle Power Saver change: Exit Time={TIME}sec", "TIME",
                  exitTime);
        parmsChanged = true;
    }

    if (parmsChanged)
    {
        if (exitUtil == 0)
        {
            // Setting the exitUtil to 0 will force restoring the default IPS
            // parmeters (0 is not valid exit utilization)
            lg2::info(
                "Idle Power Saver Exit Utilization is 0%. Restoring default parameters");
            // Read the default IPS parameters, write persistent file and update
            // DBus
            useDefaultIPSParms();
        }
        else
        {
            // Update persistant data with new DBus values
            persistedData.updateIPS(ipsEnabled, enterUtil, enterTime, exitUtil,
                                    exitTime);
        }

        // Trigger IPS data to get sent to the OCC
        sendIpsData();
    }

    return;
}

/** @brief Get the Idle Power Saver properties from persisted data
 * @return true if IPS parameters were read
 */
bool PowerMode::getIPSParms(bool& ipsEnabled, uint8_t& enterUtil,
                            uint16_t& enterTime, uint8_t& exitUtil,
                            uint16_t& exitTime)
{
    // Defaults:
    ipsEnabled = true; // Enabled
    enterUtil = 8;     // Enter Utilization (8%)
    enterTime = 240;   // Enter Delay Time (240s)
    exitUtil = 12;     // Exit Utilization (12%)
    exitTime = 10;     // Exit Delay Time (10s)

    if (!persistedData.getIPS(ipsEnabled, enterUtil, enterTime, exitUtil,
                              exitTime))
    {
        // Persistent data not initialized, read defaults and update DBus
        if (!initPersistentData())
        {
            // Unable to read defaults from entity manager yet
            return false;
        }

        persistedData.getIPS(ipsEnabled, enterUtil, enterTime, exitUtil,
                             exitTime);
    }

    if (enterUtil > exitUtil)
    {
        lg2::error(
            "ERROR: Idle Power Saver Enter Utilization ({ENTER}%) is > Exit Utilization ({EXIT}%) - using Exit for both",
            "ENTER", enterUtil, "EXIT", exitUtil);
        enterUtil = exitUtil;
    }

    return true;
}

// Set the Idle Power Saver data on DBus
bool PowerMode::updateDbusIPS(const bool enabled, const uint8_t enterUtil,
                              const uint16_t enterTime, const uint8_t exitUtil,
                              const uint16_t exitTime)
{
    if (ipsObject)
    {
        // true = skip update signal
        ipsObject->setPropertyByName(IPS_ENABLED_PROP, enabled, true);
        ipsObject->setPropertyByName(IPS_ENTER_UTIL, enterUtil, true);
        // Convert time from seconds to ms
        uint64_t msTime = enterTime * 1000;
        ipsObject->setPropertyByName(IPS_ENTER_TIME, msTime, true);
        ipsObject->setPropertyByName(IPS_EXIT_UTIL, exitUtil, true);
        msTime = exitTime * 1000;
        ipsObject->setPropertyByName(IPS_EXIT_TIME, msTime, true);
    }
    else
    {
        lg2::warning("updateDbusIPS: No IPS object was found");
    }

    return true;
}

// Send Idle Power Saver config data to the master OCC
CmdStatus PowerMode::sendIpsData()
{
    if (!masterActive || !masterOccSet)
    {
        // Nothing to do
        return CmdStatus::SUCCESS;
    }

    if (!isPowerVM())
    {
        // Idle Power Saver data is only supported on PowerVM systems
        lg2::debug(
            "PowerMode::sendIpsData: SET_CFG_DATA[IPS] does not get sent on non-PowerVM systems");
        return CmdStatus::SUCCESS;
    }

    if (!ipsObject)
    {
        // Idle Power Saver data is not available in Eco Modes
        lg2::info(
            "PowerMode::sendIpsData: Skipping IPS data due to being in an ECO Power Mode");
        return CmdStatus::SUCCESS;
    }

    bool ipsEnabled;
    uint8_t enterUtil, exitUtil;
    uint16_t enterTime, exitTime;
    getIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);

    lg2::info(
        "Idle Power Saver Parameters: enabled:{ENABLE}, enter:{EUTIL}%/{ETIME}s, exit:{XUTIL}%/{XTIME}s",
        "ENABLE", ipsEnabled, "EUTIL", enterUtil, "ETIME", enterTime, "XUTIL",
        exitUtil, "XTIME", exitTime);

    std::vector<std::uint8_t> cmd, rsp;
    cmd.reserve(12);
    cmd.push_back(uint8_t(CmdType::SET_CONFIG_DATA));
    cmd.push_back(0x00);               // Data Length (2 bytes)
    cmd.push_back(0x09);               //
    cmd.push_back(0x11);               // Config Format: IPS Settings
    cmd.push_back(0x00);               // Version
    cmd.push_back(ipsEnabled ? 1 : 0); // IPS Enable
    cmd.push_back(enterTime >> 8);     // Enter Delay Time
    cmd.push_back(enterTime & 0xFF);   //
    cmd.push_back(enterUtil);          // Enter Utilization
    cmd.push_back(exitTime >> 8);      // Exit Delay Time
    cmd.push_back(exitTime & 0xFF);    //
    cmd.push_back(exitUtil);           // Exit Utilization
    lg2::info("PowerMode::sendIpsData: SET_CFG_DATA[IPS] "
              "command to OCC{INST} ({LEN} bytes)",
              "INST", occInstance, "LEN", cmd.size());
    CmdStatus status = occCmd->send(cmd, rsp);
    if (status == CmdStatus::SUCCESS)
    {
        if (rsp.size() == 5)
        {
            if (RspStatus::SUCCESS == RspStatus(rsp[2]))
            {
                needToSendIpsData = false;
            }
            else
            {
                lg2::error(
                    "PowerMode::sendIpsData: SET_CFG_DATA[IPS] failed with status {STATUS}",
                    "STATUS", lg2::hex, rsp[2]);
                dump_hex(rsp);
                status = CmdStatus::FAILURE;
            }
        }
        else
        {
            lg2::error(
                "PowerMode::sendIpsData: INVALID SET_CFG_DATA[IPS] response");
            dump_hex(rsp);
            status = CmdStatus::FAILURE;
        }
    }
    else
    {
        lg2::error(
            "PowerMode::sendIpsData: SET_CFG_DATA[IPS] with status={STATUS}",
            "STATUS", lg2::hex, uint8_t(status));
    }

    return status;
}

// Print the current values
void OccPersistData::print()
{
    if (modeData.modeInitialized)
    {
        lg2::info(
            "OccPersistData: Mode: {MODE}, OEM Mode Data: {DATA} ({DATAHEX} Locked{LOCK})",
            "MODE", lg2::hex, uint8_t(modeData.mode), "DATA",
            modeData.oemModeData, "DATAHEX", lg2::hex, modeData.oemModeData,
            "LOCK", modeData.modeLocked);
    }
    if (modeData.ipsInitialized)
    {
        lg2::info(
            "OccPersistData: IPS enabled:{ENABLE}, enter:{EUTIL}%/{ETIME}s, exit:{XUTIL}%/{XTIME}s",
            "ENABLE", modeData.ipsEnabled, "EUTIL", modeData.ipsEnterUtil,
            "ETIME", modeData.ipsEnterTime, "XUTIL", modeData.ipsExitUtil,
            "XTIME", modeData.ipsExitTime);
    }
}

// Saves the OEM mode data in the filesystem using cereal.
void OccPersistData::save()
{
    std::filesystem::path opath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / powerModeFilename;

    if (!std::filesystem::exists(opath.parent_path()))
    {
        std::filesystem::create_directory(opath.parent_path());
    }

    lg2::debug(
        "OccPersistData::save: Writing Power Mode persisted data to {FILE}",
        "FILE", opath);
    // print();

    std::ofstream stream{opath.c_str()};
    cereal::JSONOutputArchive oarchive{stream};

    oarchive(modeData);
}

// Loads the OEM mode data in the filesystem using cereal.
void OccPersistData::load()
{
    std::filesystem::path ipath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / powerModeFilename;

    if (!std::filesystem::exists(ipath))
    {
        modeData.modeInitialized = false;
        modeData.ipsInitialized = false;
        return;
    }

    lg2::debug(
        "OccPersistData::load: Reading Power Mode persisted data from {FILE}",
        "FILE", ipath);
    try
    {
        std::ifstream stream{ipath.c_str()};
        cereal::JSONInputArchive iarchive(stream);
        iarchive(modeData);
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        lg2::error("OccPersistData::load: failed to read {FILE}, errno={ERR}",
                   "FILE", ipath, "ERR", error);
        modeData.modeInitialized = false;
        modeData.ipsInitialized = false;
    }

    // print();
}

// Called when PowerModeProperties defaults are available on DBus
void PowerMode::defaultsReady(sdbusplus::message_t& msg)
{
    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    msg.read(interface, properties);

    if (persistedData.modeAvailable())
    {
        // Validate persisted mode is supported
        SysPwrMode pMode = SysPwrMode::NO_CHANGE;
        uint16_t oemModeData = 0;
        persistedData.getMode(pMode, oemModeData);
        if (!isValidMode(pMode))
        {
            lg2::error(
                "defaultsReady: Persisted power mode ({MODE}/{DATA}) is not valid. Reading system default mode",
                "MODE", pMode, "DATA", oemModeData);
            persistedData.invalidateMode();
        }
    }

    // If persistent data exists, then don't need to read defaults
    if ((!persistedData.modeAvailable()) || (!persistedData.ipsAvailable()))
    {
        lg2::info(
            "Default PowerModeProperties are now available (persistent modeAvail={MAVAIL}, ipsAvail={IAVAIL})",
            "MAVAIL", persistedData.modeAvailable(), "IAVAIL",
            persistedData.ipsAvailable());

        // Read default power mode defaults and update DBus
        initPersistentData();
    }
}

// Get the default power mode from DBus and return true if success
bool PowerMode::getDefaultMode(SysPwrMode& defaultMode)
{
    try
    {
        auto& bus = utils::getBus();
        std::string path = "/";
        std::string service =
            utils::getServiceUsingSubTree(PMODE_DEFAULT_INTERFACE, path);
        auto method =
            bus.new_method_call(service.c_str(), path.c_str(),
                                "org.freedesktop.DBus.Properties", "Get");
        method.append(PMODE_DEFAULT_INTERFACE, "PowerMode");
        auto reply = bus.call(method);

        std::variant<std::string> stateEntryValue;
        reply.read(stateEntryValue);
        auto propVal = std::get<std::string>(stateEntryValue);

        const std::string fullModeString =
            PMODE_INTERFACE + ".PowerMode."s + propVal;
        defaultMode = powermode::convertStringToMode(fullModeString);
        if (!VALID_POWER_MODE_SETTING(defaultMode))
        {
            lg2::error("PowerMode::getDefaultMode: Invalid "
                       "default power mode found: {MODE}",
                       "MODE", defaultMode);
            // If default was read but not valid, use Max Performance
            defaultMode = SysPwrMode::MAX_PERF;
            return true;
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Unable to read Default Power Mode: {ERR}", "ERR", e.what());
        return false;
    }

    return true;
}

/* Get the default Idle Power Saver properties and return true if successful */
bool PowerMode::getDefaultIPSParms(bool& ipsEnabled, uint8_t& enterUtil,
                                   uint16_t& enterTime, uint8_t& exitUtil,
                                   uint16_t& exitTime)
{
    // Defaults:
    ipsEnabled = true; // Enabled
    enterUtil = 8;     // Enter Utilization (8%)
    enterTime = 240;   // Enter Delay Time (240s)
    exitUtil = 12;     // Exit Utilization (12%)
    exitTime = 10;     // Exit Delay Time (10s)

    std::map<std::string, std::variant<bool, uint8_t, uint16_t, uint64_t>>
        ipsProperties{};

    // Get all IPS properties from DBus
    try
    {
        auto& bus = utils::getBus();
        std::string path = "/";
        std::string service =
            utils::getServiceUsingSubTree(PMODE_DEFAULT_INTERFACE, path);
        auto method =
            bus.new_method_call(service.c_str(), path.c_str(),
                                "org.freedesktop.DBus.Properties", "GetAll");
        method.append(PMODE_DEFAULT_INTERFACE);
        auto reply = bus.call(method);
        reply.read(ipsProperties);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error(
            "Unable to read Default Idle Power Saver parameters so it will be disabled: {ERR}",
            "ERR", e.what());
        return false;
    }

    auto ipsEntry = ipsProperties.find("IdlePowerSaverEnabled");
    if (ipsEntry != ipsProperties.end())
    {
        ipsEnabled = std::get<bool>(ipsEntry->second);
    }
    else
    {
        lg2::error(
            "PowerMode::getDefaultIPSParms could not find property: IdlePowerSaverEnabled");
    }

    ipsEntry = ipsProperties.find("EnterUtilizationPercent");
    if (ipsEntry != ipsProperties.end())
    {
        enterUtil = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        lg2::error(
            "PowerMode::getDefaultIPSParms could not find property: EnterUtilizationPercent");
    }

    ipsEntry = ipsProperties.find("EnterUtilizationDwellTime");
    if (ipsEntry != ipsProperties.end())
    {
        enterTime = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        lg2::error(
            "PowerMode::getDefaultIPSParms could not find property: EnterUtilizationDwellTime");
    }

    ipsEntry = ipsProperties.find("ExitUtilizationPercent");
    if (ipsEntry != ipsProperties.end())
    {
        exitUtil = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        lg2::error(
            "PowerMode::getDefaultIPSParms could not find property: ExitUtilizationPercent");
    }

    ipsEntry = ipsProperties.find("ExitUtilizationDwellTime");
    if (ipsEntry != ipsProperties.end())
    {
        exitTime = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        lg2::error(
            "PowerMode::getDefaultIPSParms could not find property: ExitUtilizationDwellTime");
    }

    if (enterUtil > exitUtil)
    {
        lg2::error(
            "ERROR: Default Idle Power Saver Enter Utilization ({ENTER}%) is > Exit Utilization ({EXIT}%) - using Exit for both",
            "ENTER", enterUtil, "EXIT", exitUtil);
        enterUtil = exitUtil;
    }

    return true;
}

/* Read default IPS parameters, save them to the persistent file and update
 DBus. Return true if successful */
bool PowerMode::useDefaultIPSParms()
{
    // Read the default IPS parameters
    bool ipsEnabled;
    uint8_t enterUtil, exitUtil;
    uint16_t enterTime, exitTime;
    if (!getDefaultIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil,
                            exitTime))
    {
        // Unable to read defaults
        return false;
    }
    lg2::info("PowerMode::useDefaultIPSParms: Using default IPS parms: "
              "Enabled: {ENABLE}, EnterUtil: {EUTIL}%, EnterTime: {ETIME}s, "
              "ExitUtil: {XUTIL}%, ExitTime: {XTIME}s",
              "ENABLE", ipsEnabled, "EUTIL", enterUtil, "ETIME", enterTime,
              "XUTIL", exitUtil, "XTIME", exitTime);

    // Save IPS parms to the persistent file
    persistedData.updateIPS(ipsEnabled, enterUtil, enterTime, exitUtil,
                            exitTime);

    // Write IPS parms to DBus
    return updateDbusIPS(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);
}

// Starts to watch for IPS active state changes.
bool PowerMode::openIpsFile()
{
    bool rc = true;
    fd = open(ipsStatusFile.c_str(), O_RDONLY | O_NONBLOCK);
    const int open_errno = errno;
    if (fd < 0)
    {
        lg2::error("openIpsFile Error({ERR})={STR} : File={FILE}", "ERR",
                   open_errno, "STR", strerror(open_errno), "FILE",
                   ipsStatusFile);

        close(fd);

        using namespace sdbusplus::org::open_power::OCC::Device::Error;
        report<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_ERRNO(open_errno),
            phosphor::logging::org::open_power::OCC::Device::OpenFailure::
                CALLOUT_DEVICE_PATH(ipsStatusFile.c_str()));

        // We are no longer watching the error
        if (ipsObject)
        {
            ipsObject->active(false);
        }

        watching = false;
        rc = false;
        // NOTE: this will leave the system not reporting IPS active state to
        // Fan Controls, Until an APP reload, or IPL and we will attempt again.
    }
    return rc;
}

// Starts to watch for IPS active state changes.
void PowerMode::addIpsWatch(bool poll)
{
    // open file and register callback on file if we are not currently watching,
    // and if poll=true, and if we are the master.
    if ((!watching) && poll)
    {
        //  Open the file
        if (openIpsFile())
        {
            // register the callback handler which sets 'watching'
            registerIpsStatusCallBack();
        }
    }
}

// Stops watching for IPS active state changes.
void PowerMode::removeIpsWatch()
{
    //  NOTE: we want to remove event, close file, and IPS active false no
    //  matter what the 'watching' flags is set to.

    // We are no longer watching the error
    if (ipsObject)
    {
        ipsObject->active(false);
    }

    watching = false;

    // Close file
    close(fd);

    // clears sourcePtr in the event source.
    eventSource.reset();
}

// Attaches the FD to event loop and registers the callback handler
void PowerMode::registerIpsStatusCallBack()
{
    decltype(eventSource.get()) sourcePtr = nullptr;

    auto r = sd_event_add_io(event.get(), &sourcePtr, fd, EPOLLPRI | EPOLLERR,
                             ipsStatusCallBack, this);
    if (r < 0)
    {
        lg2::error("sd_event_add_io: Error({ERR})={STR} : File={FILE}", "ERR",
                   r, "STR", strerror(-r), "FILE", ipsStatusFile);

        using InternalFailure =
            sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;
        report<InternalFailure>();

        removeIpsWatch();
        // NOTE: this will leave the system not reporting IPS active state to
        // Fan Controls, Until an APP reload, or IPL and we will attempt again.
    }
    else
    {
        // puts sourcePtr in the event source.
        eventSource.reset(sourcePtr);
        // Set we are watching the error
        watching = true;
    }
}

// Static function to redirect to non static analyze event function to be
// able to read file and push onto dBus.
int PowerMode::ipsStatusCallBack(sd_event_source* /*es*/, int /*fd*/,
                                 uint32_t /*revents*/, void* userData)
{
    auto pmode = static_cast<PowerMode*>(userData);
    pmode->analyzeIpsEvent();
    return 0;
}

// Function to Read SysFs file change on IPS state and push on dBus.
void PowerMode::analyzeIpsEvent()
{
    // Need to seek to START, else the poll returns immediately telling
    // there is data to be read. if not done this floods the system.
    auto r = lseek(fd, 0, SEEK_SET);
    const int open_errno = errno;
    if (r < 0)
    {
        // NOTE: upon file access error we can not just re-open file, we have to
        // remove and add to watch.
        removeIpsWatch();
        addIpsWatch(true);
    }

    // if we are 'watching' that is the file seek, or the re-open passed.. we
    // can read the data
    if (watching)
    {
        // This file gets created when polling OCCs. A value or length of 0 is
        // deemed success. That means we would disable IPS active on dbus.
        char data;
        bool ipsState = false;
        const auto len = read(fd, &data, sizeof(data));
        const int readErrno = errno;
        if (len <= 0)
        {
            removeIpsWatch();

            lg2::error(
                "IPS state Read Error({ERR})={STR} : File={FILE} : len={LEN}",
                "ERR", readErrno, "STR", strerror(readErrno), "FILE",
                ipsStatusFile, "LEN", len);

            report<ReadFailure>(
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_ERRNO(readErrno),
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_DEVICE_PATH(ipsStatusFile.c_str()));

            // NOTE: this will leave the system not reporting IPS active state
            // to Fan Controls, Until an APP reload, or IPL and we will attempt
            // again.
        }
        else
        {
            // Data returned in ASCII.
            // convert to integer. atoi()
            // from OCC_P10_FW_Interfaces spec
            //          Bit 6: IPS active   1 indicates enabled.
            //          Bit 7: IPS enabled. 1 indicates enabled.
            //      mask off bit 6 --> & 0x02
            // Shift left one bit and store as bool. >> 1
            ipsState = static_cast<bool>(((atoi(&data)) & 0x2) >> 1);
        }

        // This will only set IPS active dbus if different than current.
        if (ipsObject)
        {
            ipsObject->active(ipsState);
        }
    }
    else
    {
        removeIpsWatch();

        // If the Retry did not get to "watching = true" we already have an
        // error log, just post trace.
        lg2::error("Retry on File seek Error({ERR})={STR} : File={FILE}", "ERR",
                   open_errno, "STR", strerror(open_errno), "FILE",
                   ipsStatusFile);

        // NOTE: this will leave the system not reporting IPS active state to
        // Fan Controls, Until an APP reload, or IPL and we will attempt again.
    }

    return;
}

// overrides read/write to powerMode dbus property.
Mode::PowerMode PowerMode::powerMode(Mode::PowerMode requestedMode)
{
    if (persistedData.getModeLock())
    {
        lg2::info("PowerMode::powerMode: mode property change blocked");
        elog<NotAllowed>(xyz::openbmc_project::Common::NotAllowed::REASON(
            "mode change not allowed due to lock"));
    }
    else
    {
        // Verify requested mode is allowed

        // Convert PowerMode to internal SysPwrMode
        SysPwrMode newMode = getInternalMode(requestedMode);
        if (newMode != SysPwrMode::NO_CHANGE)
        {
            // Validate it is an allowed customer mode
            if (customerModeList.contains(newMode))
            {
                // Update persisted data with new mode
                persistedData.updateMode(newMode, 0);

                lg2::info("DBus PowerMode Changed: {MODE}", "MODE",
                          convertPowerModeToString(requestedMode));

                // Send mode change to OCC
                sendModeChange();
            }
            else
            {
                // Not Allowed
                lg2::error(
                    "PowerMode change not allowed. {MODE} is not in AllowedPowerModes",
                    "MODE", convertPowerModeToString(requestedMode));
                elog<NotAllowed>(
                    xyz::openbmc_project::Common::NotAllowed::REASON(
                        "PowerMode value not allowed"));
            }
        }
        else
        {
            // Value is not valid
            using InvalidArgument =
                sdbusplus::xyz::openbmc_project::Common::Error::InvalidArgument;
            using Argument = xyz::openbmc_project::Common::InvalidArgument;
            lg2::error(
                "PowerMode not valid. {MODE} is not in AllowedPowerModes",
                "MODE", convertPowerModeToString(requestedMode));
            elog<InvalidArgument>(Argument::ARGUMENT_NAME("PowerMode"),
                                  Argument::ARGUMENT_VALUE("INVALID MODE"));
        }
    }

    // All elog<> calls will cause trap (so code will not make it here)

    return Mode::powerMode(requestedMode);
}

/*  Set dbus property to SAFE mode(true) or clear(false) only if different
 */
void PowerMode::updateDbusSafeMode(const bool safeModeReq)
{
    lg2::debug("PowerMode:updateDbusSafeMode: Update dbus state ({STATE})",
               "STATE", safeModeReq);

    // Note; this function checks and only updates if different.
    Mode::safeMode(safeModeReq);
}

// Get the supported power modes from DBus and return true if success
bool PowerMode::getSupportedModes()
{
    bool foundCustomerMode = false;
    using ModePropertyVariants =
        std::variant<bool, uint8_t, uint16_t, std::vector<std::string>>;
    std::map<std::string, ModePropertyVariants> powerModeProperties{};

    // Get all power mode properties from DBus
    try
    {
        auto& bus = utils::getBus();
        std::string path = "/";
        std::string service =
            utils::getServiceUsingSubTree(PMODE_DEFAULT_INTERFACE, path);
        auto method =
            bus.new_method_call(service.c_str(), path.c_str(),
                                "org.freedesktop.DBus.Properties", "GetAll");
        method.append(PMODE_DEFAULT_INTERFACE);
        auto reply = bus.call(method);
        reply.read(powerModeProperties);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Unable to read PowerModeProperties: {ERR}", "ERR",
                   e.what());
        return false;
    }

    // Determine if system suports EcoModes
    auto ecoSupport = powerModeProperties.find("EcoModeSupport");
    if (ecoSupport != powerModeProperties.end())
    {
        ecoModeSupport = std::get<bool>(ecoSupport->second);
        lg2::info("getSupportedModes(): ecoModeSupport: {SUPPORT}", "SUPPORT",
                  ecoModeSupport);
    }

    // Determine what customer modes are supported
    using PMode = sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;
    std::set<PMode::PowerMode> modesToAllow;
    auto custList = powerModeProperties.find("CustomerModes");
    if (custList != powerModeProperties.end())
    {
        auto modeList = std::get<std::vector<std::string>>(custList->second);
        for (auto mode : modeList)
        {
            // Ensure mode is valid
            const std::string fullModeString =
                PMODE_INTERFACE + ".PowerMode."s + mode;
            lg2::info("getSupportedModes(): {MODE}", "MODE", mode);
            SysPwrMode modeValue =
                powermode::convertStringToMode(fullModeString);
            if (VALID_POWER_MODE_SETTING(modeValue))
            {
                if (!foundCustomerMode)
                {
                    // Start with empty list
                    customerModeList.clear();
                    foundCustomerMode = true;
                }
                // Add mode to list
                std::optional<PMode::PowerMode> cMode =
                    PMode::convertStringToPowerMode(fullModeString);
                if (cMode)
                    modesToAllow.insert(cMode.value());
                customerModeList.insert(modeValue);
            }
            else
            {
                lg2::error(
                    "getSupportedModes(): Ignoring unsupported customer mode {MODE}",
                    "MODE", mode);
            }
        }
    }
    if (foundCustomerMode)
    {
        ModeInterface::allowedPowerModes(modesToAllow);
    }

    // Determine what OEM modes are supported
    auto oemList = powerModeProperties.find("OemModes");
    if (oemList != powerModeProperties.end())
    {
        bool foundValidMode = false;
        auto OmodeList = std::get<std::vector<std::string>>(oemList->second);
        for (auto mode : OmodeList)
        {
            // Ensure mode is valid
            const std::string fullModeString =
                PMODE_INTERFACE + ".PowerMode."s + mode;
            SysPwrMode modeValue =
                powermode::convertStringToMode(fullModeString);
            if (VALID_POWER_MODE_SETTING(modeValue) ||
                VALID_OEM_POWER_MODE_SETTING(modeValue))
            {
                if (!foundValidMode)
                {
                    // Start with empty list
                    oemModeList.clear();
                    foundValidMode = true;
                }
                // Add mode to list
                oemModeList.insert(modeValue);
            }
            else
            {
                lg2::error(
                    "getSupportedModes(): Ignoring unsupported OEM mode {MODE}",
                    "MODE", mode);
            }
        }
    }

    return foundCustomerMode;
}

bool PowerMode::isValidMode(const SysPwrMode mode)
{
    if (customerModeList.contains(mode) || oemModeList.contains(mode))
    {
        return true;
    }
    return false;
}

} // namespace powermode

} // namespace occ

} // namespace open_power
