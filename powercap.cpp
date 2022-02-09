#include "occ_status.hpp"

#include <fmt/core.h>

#include <phosphor-logging/log.hpp>
#include <powercap.hpp>

#include <cassert>

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

PowerCap::PowerCap(Status& occStatus) :
    occStatus(occStatus),
    pcapMatch(
        utils::getBus(),
        sdbusRule::member("PropertiesChanged") +
            sdbusRule::path("/xyz/openbmc_project/control/host0/power_cap") +
            sdbusRule::argN(0, "xyz.openbmc_project.Control.Power.Cap") +
            sdbusRule::interface("org.freedesktop.DBus.Properties"),
        std::bind(std::mem_fn(&PowerCap::pcapChanged), this,
                  std::placeholders::_1))
{
    pcapBasePathname = occStatus.getHwmonPath();
    if (!fs::exists(pcapBasePathname))
    {
        log<level::ERR>(fmt::format("Power Cap base filename not found: {}",
                                    pcapBasePathname.c_str())
                            .c_str());
    }
}

void PowerCap::updatePcapBounds()
{
    pcapBasePathname = occStatus.getHwmonPath();
    if (!fs::exists(pcapBasePathname))
    {
        log<level::ERR>(fmt::format("Power Cap base filename not found: {}",
                                    pcapBasePathname.c_str())
                            .c_str());
        return;
    }

    // Build the hwmon string to write the power cap bounds
    fs::path minName = getPcapFilename(std::regex{"power\\d+_cap_min$"});
    fs::path softMinName =
        getPcapFilename(std::regex{"power\\d+_cap_min_soft$"});
    fs::path maxName = getPcapFilename(std::regex{"power\\d+_cap_max$"});

    // Read the power cap bounds from sysfs files
    uint64_t cap;
    uint32_t capSoftMin = 0, capHardMin = 0, capMax = INT_MAX;
    std::ifstream softMinFile(softMinName, std::ios::in);
    if (softMinFile)
    {
        softMinFile >> cap;
        softMinFile.close();
        // Convert to Watts
        capSoftMin = cap / 1000000;
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "updatePcapBounds: unable to find pcap_min_soft file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }
    std::ifstream minFile(minName, std::ios::in);
    if (minFile)
    {
        minFile >> cap;
        minFile.close();
        // Convert to Watts
        capHardMin = cap / 1000000;
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "updatePcapBounds: unable to find cap_min file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }
    std::ifstream maxFile(maxName, std::ios::in);
    if (maxFile)
    {
        maxFile >> cap;
        maxFile.close();
        // Convert to Watts
        capMax = cap / 1000000;
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "updatePcapBounds: unable to find cap_max file: {} (errno={})",
                pcapBasePathname.c_str(), errno)
                .c_str());
    }

    // Save the bounds to dbus
    updateDbusPcap(capSoftMin, capHardMin, capMax);
}

uint32_t PowerCap::getOccInput(uint32_t pcap, bool pcapEnabled)
{
    if (!pcapEnabled)
    {
        // Pcap disabled, return 0 to indicate disabled
        return 0;
    }

    // If pcap is not disabled then just return the pcap with the derating
    // factor applied.
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
    catch (const sdbusplus::exception::exception& e)
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
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to get PowerCapEnable property",
                        entry("ERROR=%s", e.what()),
                        entry("PATH=%s", PCAP_PATH));

        return false;
    }
}

fs::path PowerCap::getPcapFilename(const std::regex& expr)
{
    for (auto& file : fs::directory_iterator(pcapBasePathname))
    {
        if (std::regex_search(file.path().string(), expr))
        {
            return file;
        }
    }
    return fs::path{};
}

void PowerCap::writeOcc(uint32_t pcapValue)
{
    // Build the hwmon string to write the user power cap
    fs::path fileName = getPcapFilename(std::regex{"power\\d+_cap_user$"});
    if (fileName.empty())
    {
        log<level::ERR>(
            fmt::format("Could not find a power cap file to write to: {})",
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
        log<level::INFO>(fmt::format("Writing {}uW to {}", pcapString.c_str(),
                                     fileName.c_str())
                             .c_str());
        file << pcapString;
        file.close();
    }
    else
    {
        log<level::ERR>(fmt::format("Failed writing {}uW to {} (errno={})",
                                    microWatts, fileName.c_str(), errno)
                            .c_str());
    }

    return;
}

void PowerCap::pcapChanged(sdbusplus::message::message& msg)
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

    bool makeChange = false;
    for (auto item : msgData)
    {
        std::string prop = item.first;
        if (prop == POWER_CAP_PROP)
        {
            pcap = std::get<uint32_t>(item.second);
            pcapEnabled = getPcapEnabled();
            makeChange = true;
        }
        else if (prop == POWER_CAP_ENABLE_PROP)
        {
            pcapEnabled = std::get<bool>(item.second);
            pcap = getPcap();
            makeChange = true;
        }
        else
        {
            // Ignore other properties
            log<level::DEBUG>(
                fmt::format(
                    "pcapChanged: Unknown power cap property changed {} to {}",
                    prop.c_str(), std::get<uint32_t>(item.second))
                    .c_str());
        }
    }

    if (makeChange)
    {
        log<level::INFO>(
            fmt::format(
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
bool PowerCap::updateDbusPcap(uint32_t& softMin, uint32_t& hardMin,
                              uint32_t& max)
{
    bool complete = true;

    try
    {
        utils::setPropertyUint32(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_SOFT_MIN,
                                 std::move(softMin));
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format(
                "updateDbusPcap: Failed to set SOFT PCAP to {}W due to {}",
                softMin, e.what())
                .c_str());
        complete = false;
    }

    try
    {
        utils::setPropertyUint32(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_HARD_MIN,
                                 std::move(hardMin));
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format(
                "updateDbusPcap: Failed to set HARD PCAP to {}W due to {}",
                hardMin, e.what())
                .c_str());
        complete = false;
    }

    try
    {
        utils::setPropertyUint32(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_MAX,
                                 std::move(max));
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>(
            fmt::format(
                "updateDbusPcap: Failed to set MAX PCAP to {}W due to {}", max,
                e.what())
                .c_str());
        complete = false;
    }

    return complete;
}
} // namespace powercap

} // namespace occ

} // namespace open_power
