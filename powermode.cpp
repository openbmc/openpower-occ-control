#include <fmt/core.h>

#include <phosphor-logging/log.hpp>
#include <powermode.hpp>
#include <xyz/openbmc_project/Control/Power/Mode/server.hpp>

#include <cassert>
#include <regex>

namespace open_power
{
namespace occ
{
namespace powermode
{

using namespace phosphor::logging;
using Mode = sdbusplus::xyz::openbmc_project::Control::Power::server::Mode;

void PowerMode::modeChanged(sdbusplus::message::message& msg)
{
    if (!occStatus.occActive())
    {
        // Nothing to do
        return;
    }

    SysPwrMode pmode = SysPwrMode::NO_CHANGE;

    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    std::string propVal;
    msg.read(interface, properties);
    const auto modeEntry = properties.find(POWER_MODE_PROP);
    if (modeEntry != properties.end())
    {
        auto modeEntryValue = modeEntry->second;
        propVal = std::get<std::string>(modeEntryValue);
        pmode = convertStringToMode(propVal);

        if (pmode != SysPwrMode::NO_CHANGE)
        {
            log<level::INFO>(
                fmt::format("Power Mode Change Requested: {}", propVal)
                    .c_str());

            // Trigger mode change to OCC
            occStatus.sendModeChange();
        }
    }

    return;
}

// Convert PowerMode string to OCC SysPwrMode
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
        pmode = SysPwrMode::DISABLE;
    }
    else
    {
        log<level::ERR>(
            fmt::format("convertStringToMode: Invalid Power Mode specified: {}",
                        i_modeString)
                .c_str());
    }

    return pmode;
}

void PowerIPS::ipsChanged(sdbusplus::message::message& msg)
{
    if (!occStatus.occActive())
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
        occStatus.sendIpsData();
    }

    return;
}

} // namespace powermode

} // namespace occ

} // namespace open_power
