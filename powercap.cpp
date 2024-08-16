#include "occ_status.hpp"

#include <phosphor-logging/log.hpp>
#include <powercap.hpp>

#include <cassert>
#include <filesystem>
#include <format>

namespace open_power
{
namespace occ
{
namespace powercap
{

constexpr auto PCAP_PATH = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";
constexpr auto POWER_CAP_SOFT_MIN = "MinSoftPowerCapValue";
constexpr auto POWER_CAP_HARD_MIN = "MinPowerCapValue";
constexpr auto POWER_CAP_MAX = "MaxPowerCapValue";

using namespace phosphor::logging;
namespace fs = std::filesystem;

void PowerCap::updatePcapBounds()
{
    // Build the hwmon string to write the power cap bounds
    fs::path minName = getPcapFilename(std::regex{"power\\d+_cap_min$"});
    fs::path softMinName =
        getPcapFilename(std::regex{"power\\d+_cap_min_soft$"});
    fs::path maxName = getPcapFilename(std::regex{"power\\d+_cap_max$"});

    // Read the current cap bounds from dbus
    uint32_t capSoftMin, capHardMin, capMax;
    readDbusPcapLimits(capSoftMin, capHardMin, capMax);

    // Read the power cap bounds from sysfs files (from OCC)
    uint64_t cap;
    std::ifstream softMinFile(softMinName, std::ios::in);
    if (softMinFile)
    {
        softMinFile >> cap;
        softMinFile.close();
        // Convert to input/AC Power in Watts (round up)
        capSoftMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
    }
    else
    {
        log<level::ERR>(
            std::format(
                "updatePcapBounds: unable to find pcap_min_soft file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }

    std::ifstream minFile(minName, std::ios::in);
    if (minFile)
    {
        minFile >> cap;
        minFile.close();
        // Convert to input/AC Power in Watts (round up)
        capHardMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
    }
    else
    {
        log<level::ERR>(
            std::format(
                "updatePcapBounds: unable to find cap_min file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }

    std::ifstream maxFile(maxName, std::ios::in);
    if (maxFile)
    {
        maxFile >> cap;
        maxFile.close();
        // Convert to input/AC Power in Watts (truncate remainder)
        capMax = cap / (PS_DERATING_FACTOR / 100.0) / 1000000;
    }
    else
    {
        log<level::ERR>(
            std::format(
                "updatePcapBounds: unable to find cap_max file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }

    // Save the power cap bounds to dbus
    updateDbusPcapLimits(capSoftMin, capHardMin, capMax);

    // Validate user power cap (if enabled) is within the bounds
    const uint32_t dbusUserCap = getPcap();
    const bool pcapEnabled = getPcapEnabled();
    if (pcapEnabled && (dbusUserCap != 0))
    {
        const uint32_t hwmonUserCap = readUserCapHwmon();
        if ((dbusUserCap >= capSoftMin) && (dbusUserCap <= capMax))
        {
            // Validate dbus and hwmon user caps match
            if ((hwmonUserCap != 0) && (dbusUserCap != hwmonUserCap))
            {
                // User power cap is enabled, but does not match dbus
                log<level::ERR>(
                    std::format(
                        "updatePcapBounds: user powercap mismatch (hwmon:{}W, bdus:{}W) - using dbus",
                        hwmonUserCap, dbusUserCap)
                        .c_str());
                auto occInput = getOccInput(dbusUserCap, pcapEnabled);
                writeOcc(occInput);
            }
        }
        else
        {
            // User power cap is outside of current bounds
            uint32_t newCap = capMax;
            if (dbusUserCap < capSoftMin)
            {
                newCap = capSoftMin;
            }
            log<level::ERR>(
                std::format(
                    "updatePcapBounds: user powercap {}W is outside bounds "
                    "(soft min:{}, min:{}, max:{})",
                    dbusUserCap, capSoftMin, capHardMin, capMax)
                    .c_str());
            try
            {
                log<level::INFO>(
                    std::format(
                        "updatePcapBounds: Updating user powercap from {} to {}W",
                        hwmonUserCap, newCap)
                        .c_str());
                utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP,
                                   newCap);
                auto occInput = getOccInput(newCap, pcapEnabled);
                writeOcc(occInput);
            }
            catch (const sdbusplus::exception_t& e)
            {
                log<level::ERR>(
                    std::format(
                        "updatePcapBounds: Failed to update user powercap due to ",
                        e.what())
                        .c_str());
            }
        }
    }
}

// Get value of power cap to send to the OCC (output/DC power)
uint32_t PowerCap::getOccInput(uint32_t pcap, bool pcapEnabled)
{
    if (!pcapEnabled)
    {
        // Pcap disabled, return 0 to indicate disabled
        return 0;
    }

    // If pcap is not disabled then just return the pcap with the derating
    // factor applied (output/DC power).
    return ((static_cast<uint64_t>(pcap) * PS_DERATING_FACTOR) / 100);
}

uint32_t PowerCap::getPcap()
{
    utils::PropertyValue pcap{};
    try
    {
        pcap = utils::getProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP);

        return std::get<uint32_t>(pcap);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>("Failed to get PowerCap property",
                        entry("ERROR=%s", e.what()),
                        entry("PATH=%s", PCAP_PATH));

        return 0;
    }
}

bool PowerCap::getPcapEnabled()
{
    utils::PropertyValue pcapEnabled{};
    try
    {
        pcapEnabled = utils::getProperty(PCAP_PATH, PCAP_INTERFACE,
                                         POWER_CAP_ENABLE_PROP);

        return std::get<bool>(pcapEnabled);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>("Failed to get PowerCapEnable property",
                        entry("ERROR=%s", e.what()),
                        entry("PATH=%s", PCAP_PATH));

        return false;
    }
}

fs::path PowerCap::getPcapFilename(const std::regex& expr)
{
    if (pcapBasePathname.empty())
    {
        pcapBasePathname = occStatus.getHwmonPath();
    }

    if (fs::exists(pcapBasePathname))
    {
        // Search for pcap file based on the supplied expr
        for (auto& file : fs::directory_iterator(pcapBasePathname))
        {
            if (std::regex_search(file.path().string(), expr))
            {
                // Found match
                return file;
            }
        }
    }
    else
    {
        log<level::ERR>(std::format("Power Cap base filename not found: {}",
                                    pcapBasePathname.c_str())
                            .c_str());
    }

    // return empty path
    return fs::path{};
}

// Write the user power cap to sysfs (output/DC power)
// This will trigger the driver to send the cap to the OCC
void PowerCap::writeOcc(uint32_t pcapValue)
{
    if (!occStatus.occActive())
    {
        // OCC not running, skip update
        return;
    }

    // Build the hwmon string to write the user power cap
    fs::path fileName = getPcapFilename(std::regex{"power\\d+_cap_user$"});
    if (fileName.empty())
    {
        log<level::ERR>(
            std::format("Could not find a power cap file to write to: {})",
                        pcapBasePathname.c_str())
                .c_str());
        return;
    }

    uint64_t microWatts = pcapValue * 1000000ull;

    auto pcapString{std::to_string(microWatts)};

    // Open the hwmon file and write the power cap
    std::ofstream file(fileName, std::ios::out);
    if (file)
    {
        log<level::INFO>(std::format("Writing {}uW to {}", pcapString.c_str(),
                                     fileName.c_str())
                             .c_str());
        file << pcapString;
        file.close();
    }
    else
    {
        log<level::ERR>(std::format("Failed writing {}uW to {} (errno={})",
                                    microWatts, fileName.c_str(), errno)
                            .c_str());
    }

    return;
}

// Read the current user power cap from sysfs as input/AC power
uint32_t PowerCap::readUserCapHwmon()
{
    uint32_t userCap = 0;

    // Get the sysfs filename for the user power cap
    fs::path userCapName = getPcapFilename(std::regex{"power\\d+_cap_user$"});
    if (userCapName.empty())
    {
        log<level::ERR>(
            std::format(
                "readUserCapHwmon: Could not find a power cap file to read: {})",
                pcapBasePathname.c_str())
                .c_str());
        return 0;
    }

    // Open the sysfs file and read the power cap
    std::ifstream file(userCapName, std::ios::in);
    if (file)
    {
        uint64_t cap;
        file >> cap;
        file.close();
        // Convert to input/AC Power in Watts
        userCap = (cap / (PS_DERATING_FACTOR / 100.0) / 1000000);
    }
    else
    {
        log<level::ERR>(
            std::format("readUserCapHwmon: Failed reading {} (errno={})",
                        userCapName.c_str(), errno)
                .c_str());
    }

    return userCap;
}

void PowerCap::pcapChanged(sdbusplus::message_t& msg)
{
    if (!occStatus.occActive())
    {
        // Nothing to do
        return;
    }

    uint32_t pcap = 0;
    bool pcapEnabled = false;

    std::string msgSensor;
    std::map<std::string, std::variant<uint32_t, bool>> msgData;
    msg.read(msgSensor, msgData);

    bool changeFound = false;
    for (const auto& [prop, value] : msgData)
    {
        if (prop == POWER_CAP_PROP)
        {
            pcap = std::get<uint32_t>(value);
            pcapEnabled = getPcapEnabled();
            changeFound = true;
        }
        else if (prop == POWER_CAP_ENABLE_PROP)
        {
            pcapEnabled = std::get<bool>(value);
            pcap = getPcap();
            changeFound = true;
        }
        else
        {
            // Ignore other properties
            log<level::DEBUG>(
                std::format(
                    "pcapChanged: Unknown power cap property changed {} to {}",
                    prop.c_str(), std::get<uint32_t>(value))
                    .c_str());
        }
    }

    // Validate the cap is within supported range
    uint32_t capSoftMin, capHardMin, capMax;
    readDbusPcapLimits(capSoftMin, capHardMin, capMax);
    if (((pcap > 0) && (pcap < capSoftMin)) || ((pcap == 0) && (pcapEnabled)))
    {
        log<level::ERR>(
            std::format(
                "pcapChanged: Power cap of {}W is lower than allowed (soft min:{}, min:{}) - using soft min",
                pcap, capSoftMin, capHardMin)
                .c_str());
        pcap = capSoftMin;
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP, pcap);
    }
    else if (pcap > capMax)
    {
        log<level::ERR>(
            std::format(
                "pcapChanged: Power cap of {}W is higher than allowed (max:{}) - using max",
                pcap, capSoftMin, capHardMin)
                .c_str());
        pcap = capMax;
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP, pcap);
    }

    if (changeFound)
    {
        log<level::INFO>(
            std::format(
                "Power Cap Property Change (cap={}W (input), enabled={})", pcap,
                pcapEnabled ? 'y' : 'n')
                .c_str());

        // Determine desired action to write to occ
        auto occInput = getOccInput(pcap, pcapEnabled);
        // Write action to occ
        writeOcc(occInput);
    }

    return;
}

// Update the Power Cap bounds on DBus
bool PowerCap::updateDbusPcapLimits(uint32_t softMin, uint32_t hardMin,
                                    uint32_t max)
{
    bool complete = true;

    try
    {
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_SOFT_MIN,
                           softMin);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format(
                "updateDbusPcapLimits: Failed to set SOFT PCAP to {}W due to {}",
                softMin, e.what())
                .c_str());
        complete = false;
    }

    try
    {
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_HARD_MIN,
                           hardMin);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format(
                "updateDbusPcapLimits: Failed to set HARD PCAP to {}W due to {}",
                hardMin, e.what())
                .c_str());
        complete = false;
    }

    try
    {
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_MAX, max);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format(
                "updateDbusPcapLimits: Failed to set MAX PCAP to {}W due to {}",
                max, e.what())
                .c_str());
        complete = false;
    }

    return complete;
}

// Read the Power Cap bounds from DBus
bool PowerCap::readDbusPcapLimits(uint32_t& softMin, uint32_t& hardMin,
                                  uint32_t& max)
{
    bool complete = true;
    utils::PropertyValue prop{};

    try
    {
        prop =
            utils::getProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_SOFT_MIN);
        softMin = std::get<uint32_t>(prop);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format("readDbusPcapLimits: Failed to get SOFT PCAP due to {}",
                        e.what())
                .c_str());
        softMin = 0;
        complete = false;
    }

    try
    {
        prop =
            utils::getProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_HARD_MIN);
        hardMin = std::get<uint32_t>(prop);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format("readDbusPcapLimits: Failed to get HARD PCAP due to {}",
                        e.what())
                .c_str());
        hardMin = 0;
        complete = false;
    }

    try
    {
        prop = utils::getProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_MAX);
        max = std::get<uint32_t>(prop);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            std::format("readDbusPcapLimits: Failed to get MAX PCAP due to {}",
                        e.what())
                .c_str());
        max = INT_MAX;
        complete = false;
    }

    return complete;
}

} // namespace powercap

} // namespace occ

} // namespace open_power
