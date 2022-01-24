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
using Mode = sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;

// Set the Master OCC
void PowerMode::setMasterOcc(const std::string& occPath)
{
    path = occPath;
    occInstance = path.back() - '0';
    log<level::DEBUG>(fmt::format("PowerMode::setMasterOcc(OCC{}, {})",
                                  occInstance, path.c_str())
                          .c_str());
    occCmd = std::make_unique<open_power::occ::OccCommand>(occInstance,
                                                           path.c_str());
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
            // DBus mode changed, get rid of any OEM mode if set
            persistedData.purge();

            log<level::INFO>(
                fmt::format("Power Mode Change Requested: {}", propVal)
                    .c_str());

            // Send mode change to OCC
            sendModeChange();
        }
    }
}

// Called from OCC PassThrough interface (via CE login / BMC command line)
bool PowerMode::setMode(const SysPwrMode newMode, const uint16_t modeData)
{
    if (updateDbusMode(newMode) == false)
    {
        // Unsupported mode
        return false;
    }

    // If new mode is valid customer mode, the DBus update will trigger the mode
    // change request to OCC.  For OEM modes, the request will be sent here.
    if (VALID_OEM_POWER_MODE_SETTING(newMode))
    {
        // Save OEM mode
        persistedData.writeModeFile(newMode, modeData);

        // Send mode change to OCC
        if (sendModeChange() != CmdStatus::SUCCESS)
        {
            // Mode change failed
            return false;
        }
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
        fmt::format("isPowerVM returning {}", powerVmTarget).c_str());

    return powerVmTarget;
}

// Get the requested power mode from DBus
SysPwrMode PowerMode::getDbusMode()
{
    using namespace open_power::occ::powermode;
    SysPwrMode currentMode;

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

    currentMode = powermode::convertStringToMode(propVal);
    if (!VALID_POWER_MODE_SETTING(currentMode))
    {
        log<level::ERR>(
            fmt::format(
                "PowerMode::getDbusMode Invalid power mode found on DBus: {}",
                currentMode)
                .c_str());
        currentMode = SysPwrMode::NO_CHANGE;
    }

    return currentMode;
}

// Set the power mode on DBus
bool PowerMode::updateDbusMode(const SysPwrMode newMode)
{
    using namespace open_power::occ::powermode;
    using namespace std::literals::string_literals;

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

    // Mode::PowerMode dBusMode;
    std::string dBusMode;
    switch (newMode)
    {
        case SysPwrMode::STATIC:
            dBusMode = PMODE_INTERFACE + ".PowerMode.Static"s;
            break;
        case SysPwrMode::POWER_SAVING:
            dBusMode = PMODE_INTERFACE + ".PowerMode.PowerSaving"s;
            break;
        case SysPwrMode::MAX_PERF:
            dBusMode = PMODE_INTERFACE + ".PowerMode.MaximumPerformance"s;
            break;
        default:
            dBusMode = PMODE_INTERFACE + ".PowerMode.OEM"s;
    }

    utils::setProperty(PMODE_PATH, PMODE_INTERFACE, POWER_MODE_PROP,
                       std::move(dBusMode));

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

    // Use OEM power mode if it was set
    SysPwrMode newMode = SysPwrMode::NO_CHANGE;
    uint16_t modeData = 0;
    if (persistedData.getOemMode(newMode, modeData) == false)
    {
        // Read customer power mode from Dbus
        newMode = getDbusMode();
    }

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
        cmd.push_back(modeData >> 8);   // Mode Data (Freq Point)
        cmd.push_back(modeData & 0xFF); //
        cmd.push_back(0x00);            // reserved
        log<level::INFO>(
            fmt::format(
                "PowerMode::sendModeChange: SET_MODE({},{}) command to OCC{} ({} bytes)",
                newMode, modeData, occInstance, cmd.size())
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
            if (status == CmdStatus::OPEN_FAILURE)
            {
                // OCC not active yet
                status = CmdStatus::SUCCESS;
            }
            else
            {
                log<level::ERR>("PowerMode::sendModeChange: SET_MODE FAILED!");
            }
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
    if (!masterActive || !masterOccSet)
    {
        // Nothing to do
        return;
    }

    bool parmsChanged = false;
    std::string interface;
    std::map<std::string, std::variant<bool, uint8_t, uint64_t>>
        ipsProperties{};
    msg.read(interface, ipsProperties);

    auto ipsEntry = ipsProperties.find(IPS_ENABLED_PROP);
    if (ipsEntry != ipsProperties.end())
    {
        const auto ipsEnabled = std::get<bool>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enabled={}", ipsEnabled)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        const auto enterUtil = std::get<uint8_t>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enter Util={}%", enterUtil)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_ENTER_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        const auto enterTime =
            std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Enter Time={}sec", enterTime)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        const auto exitUtil = std::get<uint8_t>(ipsEntry->second);
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Exit Util={}%", exitUtil)
                .c_str());
        parmsChanged = true;
    }
    ipsEntry = ipsProperties.find(IPS_EXIT_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        const auto exitTime =
            std::chrono::duration_cast<std::chrono::seconds>(ms).count();
        log<level::INFO>(
            fmt::format("Idle Power Saver change: Exit Time={}sec", exitTime)
                .c_str());
        parmsChanged = true;
    }

    if (parmsChanged)
    {
        // Trigger mode change to OCC
        sendIpsData();
    }

    return;
}

/** @brief Get the Idle Power Saver properties
 * @return true if IPS is enabled
 */
bool PowerMode::getIPSParms(uint8_t& enterUtil, uint16_t& enterTime,
                            uint8_t& exitUtil, uint16_t& exitTime)
{
    using namespace open_power::occ::powermode;
    // Defaults:
    bool ipsEnabled = false; // Disabled
    enterUtil = 8;           // Enter Utilization (8%)
    enterTime = 240;         // Enter Delay Time (240s)
    exitUtil = 12;           // Exit Utilization (12%)
    exitTime = 10;           // Exit Delay Time (10s)

    std::map<std::string, std::variant<bool, uint8_t, uint64_t>>
        ipsProperties{};

    // Get all IPS properties from DBus
    try
    {
        auto& bus = utils::getBus();
        auto service = utils::getService(PIPS_PATH, PIPS_INTERFACE);
        auto method =
            bus.new_method_call(service.c_str(), PIPS_PATH,
                                "org.freedesktop.DBus.Properties", "GetAll");
        method.append(PIPS_INTERFACE);
        auto reply = bus.call(method);
        reply.read(ipsProperties);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format(
                "Unable to read Idle Power Saver parameters so it will be disabled: {}",
                e.what())
                .c_str());
        return ipsEnabled;
    }

    auto ipsEntry = ipsProperties.find(IPS_ENABLED_PROP);
    if (ipsEntry != ipsProperties.end())
    {
        ipsEnabled = std::get<bool>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            fmt::format("PowerMode::getIPSParms could not find property: {}",
                        IPS_ENABLED_PROP)
                .c_str());
    }

    ipsEntry = ipsProperties.find(IPS_ENTER_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        enterUtil = std::get<uint8_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            fmt::format("PowerMode::getIPSParms could not find property: {}",
                        IPS_ENTER_UTIL)
                .c_str());
    }

    ipsEntry = ipsProperties.find(IPS_ENTER_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        enterTime =
            std::chrono::duration_cast<std::chrono::seconds>(ms).count();
    }
    else
    {
        log<level::ERR>(
            fmt::format("PowerMode::getIPSParms could not find property: {}",
                        IPS_ENTER_TIME)
                .c_str());
    }

    ipsEntry = ipsProperties.find(IPS_EXIT_UTIL);
    if (ipsEntry != ipsProperties.end())
    {
        exitUtil = std::get<uint8_t>(ipsEntry->second);
    }
    else
    {
        log<level::ERR>(
            fmt::format("PowerMode::getIPSParms could not find property: {}",
                        IPS_EXIT_UTIL)
                .c_str());
    }

    ipsEntry = ipsProperties.find(IPS_EXIT_TIME);
    if (ipsEntry != ipsProperties.end())
    {
        std::chrono::milliseconds ms(std::get<uint64_t>(ipsEntry->second));
        exitTime = std::chrono::duration_cast<std::chrono::seconds>(ms).count();
    }
    else
    {
        log<level::ERR>(
            fmt::format("PowerMode::getIPSParms could not find property: {}",
                        IPS_EXIT_TIME)
                .c_str());
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

    return ipsEnabled;
}

// Send Idle Power Saver config data to the master OCC
CmdStatus PowerMode::sendIpsData()
{
    CmdStatus status;

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

    uint8_t enterUtil, exitUtil;
    uint16_t enterTime, exitTime;
    const bool ipsEnabled =
        getIPSParms(enterUtil, enterTime, exitUtil, exitTime);

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
    status = occCmd->send(cmd, rsp);
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
        if (status == CmdStatus::OPEN_FAILURE)
        {
            // OCC not active yet
            status = CmdStatus::SUCCESS;
        }
        else
        {
            log<level::ERR>(
                "PowerMode::sendIpsData: SET_CFG_DATA[IPS] FAILED!");
        }
    }

    return status;
}

inline void OccPersistData::print()
{
    log<level::DEBUG>(
        fmt::format(
            "OccPersistData: OEM Mode: 0x{:02X}, OEM Mode Freq: {} (0x{:04X})",
            oemData.oemMode, oemData.oemModeFreq, oemData.oemModeFreq)
            .c_str());
}

// Saves the OEM mode data in the filesystem using cereal.
void OccPersistData::save()
{
    std::filesystem::path opath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / oemModeFilename;

    if (!std::filesystem::exists(opath.parent_path()))
    {
        std::filesystem::create_directory(opath.parent_path());
    }

    log<level::DEBUG>(
        fmt::format("OccPersistData::save: Writing OEM persisted data to {}",
                    opath.c_str())
            .c_str());
    print();

    std::ofstream stream{opath.c_str()};
    cereal::JSONOutputArchive oarchive{stream};

    oarchive(oemData);
}

// Loads the OEM mode data in the filesystem using cereal.
void OccPersistData::load()
{

    std::filesystem::path ipath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / oemModeFilename;

    if (!std::filesystem::exists(ipath))
    {
        return;
    }

    log<level::DEBUG>(
        fmt::format("OccPersistData::load: Reading OEM persisted data from {}",
                    ipath.c_str())
            .c_str());
    try
    {
        std::ifstream stream{ipath.c_str()};
        cereal::JSONInputArchive iarchive(stream);
        iarchive(oemData);

        oemSet = true;
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        log<level::ERR>(
            fmt::format("OccPersistData::load: failed to read {}, errno={}",
                        ipath.c_str(), error)
                .c_str());
    }

    print();
}

void OccPersistData::purge()
{
    std::filesystem::path opath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / oemModeFilename;

    if (!std::filesystem::exists(opath))
    {
        return;
    }

    print();
    log<level::DEBUG>("OccPersistData::purge() Removing OEM data");

    oemSet = false;
    oemData.oemMode = SysPwrMode::NO_CHANGE;
    oemData.oemModeFreq = 0x0000;
    remove(opath.c_str());
}

} // namespace powermode

} // namespace occ

} // namespace open_power
