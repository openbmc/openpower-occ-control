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
        // Nothing to  do
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

} // namespace powermode

} // namespace occ

} // namespace open_power
