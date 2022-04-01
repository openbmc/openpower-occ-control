#include "powermode.hpp"

#include <fmt/core.h>

#include <com/ibm/Host/Target/server.hpp>
#include <phosphor-logging/log.hpp>
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

using Mode = sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;

// Set the Master OCC
void PowerMode::setMasterOcc(const std::string& masterOccPath)
{
    if (masterOccSet)
    {
        if (masterOccPath != path)
        {
            log<level::ERR>(
                fmt::format(
                    "PowerMode::setMasterOcc: Master changed (was OCC{}, {})",
                    occInstance, masterOccPath)
                    .c_str());
            if (occCmd)
            {
                occCmd.reset();
            }
        }
    }
    path = masterOccPath;
    occInstance = path.back() - '0';
    log<level::DEBUG>(fmt::format("PowerMode::setMasterOcc(OCC{}, {})",
                                  occInstance, path.c_str())
                          .c_str());
    if (!occCmd)
    {
        occCmd = std::make_unique<open_power::occ::OccCommand>(occInstance,
                                                               path.c_str());
    }
    masterOccSet = true;
};

// Called when DBus power mode gets changed
void PowerMode::modeChanged(sdbusplus::message::message& msg)
{
    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    std::string propVal;
    msg.read(interface, properties);
    const auto modeEntry = properties.find(POWER_MODE_PROP);
    if (modeEntry != properties.end())
    {
        auto modeEntryValue = modeEntry->second;
        propVal = std::get<std::string>(modeEntryValue);
        SysPwrMode newMode = convertStringToMode(propVal);
        if (newMode != SysPwrMode::NO_CHANGE)
        {
            // Update persisted data with new mode
            persistedData.updateMode(newMode, 0);

            log<level::INFO>(
                fmt::format("DBus Power Mode Changed: {}", propVal).c_str());

            // Send mode change to OCC
            sendModeChange();
        }
    }
}

// Called from OCC PassThrough interface (via CE login / BMC command line)
bool PowerMode::setMode(const SysPwrMode newMode, const uint16_t oemModeData)
{
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

// Convert PowerMode string to OCC SysPwrMode
// Returns NO_CHANGE if OEM or unsupported mode
SysPwrMode convertStringToMode(const std::string& i_modeString)
{
    SysPwrMode pmode = SysPwrMode::NO_CHANGE;

    Mode::PowerMode mode = Mode::convertPowerModeFromString(i_modeString);
    if (mode == Mode::PowerMode::MaximumPerformance)
    {
        pmode = SysPwrMode::MAX_PERF;
    }
    else if (mode == Mode::PowerMode::PowerSaving)
    {
        pmode = SysPwrMode::POWER_SAVING;
    }
    else if (mode == Mode::PowerMode::Static)
    {
        pmode = SysPwrMode::STATIC;
    }
    else
    {
        if (mode != Mode::PowerMode::OEM)
        {
            log<level::ERR>(
                fmt::format(
                    "convertStringToMode: Invalid Power Mode specified: {}",
                    i_modeString)
                    .c_str());
        }
    }

    return pmode;
}

// Check if Hypervisor target is PowerVM
bool isPowerVM()
{
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
        fmt::format("isPowerVM returning {}", powerVmTarget).c_str());

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
        log<level::INFO>(
            fmt::format("PowerMode::initPersistentData: Using default mode: {}",
                        currentMode)
                .c_str());

        // Save default mode as current mode
        persistedData.updateMode(currentMode, 0);

        // Write default mode to DBus
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

// Set the power mode on DBus
bool PowerMode::updateDbusMode(const SysPwrMode newMode)
{
    if (!VALID_POWER_MODE_SETTING(newMode) &&
        !VALID_OEM_POWER_MODE_SETTING(newMode))
    {
        log<level::ERR>(
            fmt::format(
                "PowerMode::updateDbusMode - Requested power mode not supported: {}",
                newMode)
                .c_str());
        return false;
    }

    // Convert mode for DBus
    ModeInterface::PowerMode dBusMode;
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
        default:
            dBusMode = Mode::PowerMode::OEM;
    }

    // true = skip update signal
    ModeInterface::setPropertyByName(POWER_MODE_PROP, dBusMode, true);

    return true;
}

// Send mode change request to the master OCC
CmdStatus PowerMode::sendModeChange()
{
    CmdStatus status;

    if (!masterActive || !masterOccSet)
    {
        // Nothing to do
        log<level::DEBUG>("PowerMode::sendModeChange: OCC master not active");
        return CmdStatus::SUCCESS;
    }

    if (!isPowerVM())
    {
        // Mode change is only supported on PowerVM systems
        log<level::DEBUG>(
            "PowerMode::sendModeChange: MODE CHANGE does not get sent on non-PowerVM systems");
        return CmdStatus::SUCCESS;
    }

    SysPwrMode newMode;
    uint16_t oemModeData = 0;
    getMode(newMode, oemModeData);

    if (VALID_POWER_MODE_SETTING(newMode) ||
        VALID_OEM_POWER_MODE_SETTING(newMode))
    {
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
        log<level::INFO>(
            fmt::format(
                "PowerMode::sendModeChange: SET_MODE({},{}) command to OCC{} ({} bytes)",
                newMode, oemModeData, occInstance, cmd.size())
                .c_str());
        status = occCmd->send(cmd, rsp);
        if (status == CmdStatus::SUCCESS)
        {
            if (rsp.size() == 5)
            {
                if (RspStatus::SUCCESS != RspStatus(rsp[2]))
                {
                    log<level::ERR>(
                        fmt::format(
                            "PowerMode::sendModeChange: SET MODE failed with status 0x{:02X}",
                            rsp[2])
                            .c_str());
                    dump_hex(rsp);
                    status = CmdStatus::FAILURE;
                }
            }
            else
            {
                log<level::ERR>(
                    "PowerMode::sendModeChange: INVALID SET MODE response");
                dump_hex(rsp);
                status = CmdStatus::FAILURE;
            }
        }
        else
        {
            log<level::ERR>(
                fmt::format(
                    "PowerMode::sendModeChange: SET_MODE FAILED with status={}",
                    status)
                    .c_str());
        }
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "PowerMode::sendModeChange: Unable to set power mode to {}",
                newMode)
                .c_str());
        status = CmdStatus::FAILURE;
    }

    return status;
}

void PowerMode::ipsChanged(sdbusplus::message::message& msg)
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

    // Check for any changed data
    auto ipsEntry = ipsProperties.find(IPS_ENABLED_PROP);
    if (ipsEntry != ipsProperties.end())
    {
        ipsEnabled = std::get<bool>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enabled={}", ipsEnabled)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        enterUtil = std::get<uint8_t>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enter Util={}%", enterUtil)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        enterTime =
            std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enter Time={}sec", enterTime)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        exitUtil = std::get<uint8_t>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Exit Util={}%", exitUtil)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        exitTime = std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Exit Time={}sec", exitTime)
                .c_str());
        parmsChanged = true;
    }

    if (parmsChanged)
    {
        if (exitUtil == 0)
        {
            // Setting the exitUtil to 0 will force restoring the default IPS
            // parmeters (0 is not valid exit utilization)
            log<level::INFO>(
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
        log<level::ERR>(
            fmt::format(
                "ERROR: Idle Power Saver Enter Utilization ({}%) is > Exit Utilization ({}%) - using Exit for both",
                enterUtil, exitUtil)
                .c_str());
        enterUtil = exitUtil;
    }

    return true;
}

// Set the Idle Power Saver data on DBus
bool PowerMode::updateDbusIPS(const bool enabled, const uint8_t enterUtil,
                              const uint16_t enterTime, const uint8_t exitUtil,
                              const uint16_t exitTime)
{
    // true = skip update signal
    IpsInterface::setPropertyByName(IPS_ENABLED_PROP, enabled, true);
    IpsInterface::setPropertyByName(IPS_ENTER_UTIL, enterUtil, true);
    // Convert time from seconds to ms
    uint64_t msTime = enterTime * 1000;
    IpsInterface::setPropertyByName(IPS_ENTER_TIME, msTime, true);
    IpsInterface::setPropertyByName(IPS_EXIT_UTIL, exitUtil, true);
    msTime = exitTime * 1000;
    IpsInterface::setPropertyByName(IPS_EXIT_TIME, msTime, true);

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
        log<level::DEBUG>(
            "PowerMode::sendIpsData: SET_CFG_DATA[IPS] does not get sent on non-PowerVM systems");
        return CmdStatus::SUCCESS;
    }

    bool ipsEnabled;
    uint8_t enterUtil, exitUtil;
    uint16_t enterTime, exitTime;
    getIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);

    log<level::INFO>(
        fmt::format(
            "Idle Power Saver Parameters: enabled:{}, enter:{}%/{}s, exit:{}%/{}s",
            ipsEnabled, enterUtil, enterTime, exitUtil, exitTime)
            .c_str());

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
    log<level::INFO>(fmt::format("PowerMode::sendIpsData: SET_CFG_DATA[IPS] "
                                 "command to OCC{} ({} bytes)",
                                 occInstance, cmd.size())
                         .c_str());
    CmdStatus status = occCmd->send(cmd, rsp);
    if (status == CmdStatus::SUCCESS)
    {
        if (rsp.size() == 5)
        {
            if (RspStatus::SUCCESS != RspStatus(rsp[2]))
            {
                log<level::ERR>(
                    fmt::format(
                        "PowerMode::sendIpsData: SET_CFG_DATA[IPS] failed with status 0x{:02X}",
                        rsp[2])
                        .c_str());
                dump_hex(rsp);
                status = CmdStatus::FAILURE;
            }
        }
        else
        {
            log<level::ERR>(
                "PowerMode::sendIpsData: INVALID SET_CFG_DATA[IPS] response");
            dump_hex(rsp);
            status = CmdStatus::FAILURE;
        }
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "PowerMode::sendIpsData: SET_CFG_DATA[IPS] with status={}",
                status)
                .c_str());
    }

    return status;
}

// Print the current values
void OccPersistData::print()
{
    if (modeData.modeInitialized)
    {
        log<level::INFO>(
            fmt::format(
                "OccPersistData: Mode: 0x{:02X}, OEM Mode Data: {} (0x{:04X})",
                modeData.mode, modeData.oemModeData, modeData.oemModeData)
                .c_str());
    }
    if (modeData.ipsInitialized)
    {
        log<level::INFO>(
            fmt::format(
                "OccPersistData: IPS enabled:{}, enter:{}%/{}s, exit:{}%/{}s",
                modeData.ipsEnabled, modeData.ipsEnterUtil,
                modeData.ipsEnterTime, modeData.ipsExitUtil,
                modeData.ipsExitTime)
                .c_str());
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

    log<level::DEBUG>(
        fmt::format(
            "OccPersistData::save: Writing Power Mode persisted data to {}",
            opath.c_str())
            .c_str());
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

    log<level::DEBUG>(
        fmt::format(
            "OccPersistData::load: Reading Power Mode persisted data from {}",
            ipath.c_str())
            .c_str());
    try
    {
        std::ifstream stream{ipath.c_str()};
        cereal::JSONInputArchive iarchive(stream);
        iarchive(modeData);
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        log<level::ERR>(
            fmt::format("OccPersistData::load: failed to read {}, errno={}",
                        ipath.c_str(), error)
                .c_str());
        modeData.modeInitialized = false;
        modeData.ipsInitialized = false;
    }

    // print();
}

// Called when PowerModeProperties defaults are available on DBus
void PowerMode::defaultsReady(sdbusplus::message::message& msg)
{
    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    msg.read(interface, properties);

    // If persistent data exists, then don't need to read defaults
    if ((!persistedData.modeAvailable()) || (!persistedData.ipsAvailable()))
    {
        log<level::INFO>(
            fmt::format(
                "Default PowerModeProperties are now available (persistent modeAvail={}, ipsAvail={})",
                persistedData.modeAvailable() ? 'y' : 'n',
                persistedData.modeAvailable() ? 'y' : 'n')
                .c_str());

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
            log<level::ERR>(
                fmt::format(
                    "PowerMode::getDefaultMode: Invalid default power mode found: {}",
                    defaultMode)
                    .c_str());
            // If default was read but not valid, use Max Performance
            defaultMode = SysPwrMode::MAX_PERF;
            return true;
        }
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format("Unable to read Default Power Mode: {}", e.what())
                .c_str());
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
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format(
                "Unable to read Default Idle Power Saver parameters so it will be disabled: {}",
                e.what())
                .c_str());
        return false;
    }

    auto ipsEntry = ipsProperties.find("IdlePowerSaverEnabled");
    if (ipsEntry != ipsProperties.end())
    {
        ipsEnabled = std::get<bool>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            "PowerMode::getDefaultIPSParms could not find property: IdlePowerSaverEnabled");
    }

    ipsEntry = ipsProperties.find("EnterUtilizationPercent");
    if (ipsEntry != ipsProperties.end())
    {
        enterUtil = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            "PowerMode::getDefaultIPSParms could not find property: EnterUtilizationPercent");
    }

    ipsEntry = ipsProperties.find("EnterUtilizationDwellTime");
    if (ipsEntry != ipsProperties.end())
    {
        enterTime = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            "PowerMode::getDefaultIPSParms could not find property: EnterUtilizationDwellTime");
    }

    ipsEntry = ipsProperties.find("ExitUtilizationPercent");
    if (ipsEntry != ipsProperties.end())
    {
        exitUtil = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            "PowerMode::getDefaultIPSParms could not find property: ExitUtilizationPercent");
    }

    ipsEntry = ipsProperties.find("ExitUtilizationDwellTime");
    if (ipsEntry != ipsProperties.end())
    {
        exitTime = std::get<uint64_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            "PowerMode::getDefaultIPSParms could not find property: ExitUtilizationDwellTime");
    }

    if (enterUtil > exitUtil)
    {
        log<level::ERR>(
            fmt::format(
                "ERROR: Default Idle Power Saver Enter Utilization ({}%) is > Exit Utilization ({}%) - using Exit for both",
                enterUtil, exitUtil)
                .c_str());
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
    log<level::INFO>(
        fmt::format(
            "PowerMode::useDefaultIPSParms: Using default IPS parms: Enabled: {}, EnterUtil: {}%, EnterTime: {}s, ExitUtil: {}%, ExitTime: {}s",
            ipsEnabled, enterUtil, enterTime, exitUtil, exitTime)
            .c_str());

    // Save IPS parms to the persistent file
    persistedData.updateIPS(ipsEnabled, enterUtil, enterTime, exitUtil,
                            exitTime);

    // Write IPS parms to DBus
    return updateDbusIPS(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);
}

} // namespace powermode

} // namespace occ

} // namespace open_power
