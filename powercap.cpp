#include <phosphor-logging/log.hpp>
#include <powercap.hpp>

#include <cassert>
#include <regex>

namespace open_power
{
namespace occ
{
namespace powercap
{

constexpr auto PCAP_PATH = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";

using namespace phosphor::logging;
namespace fs = std::experimental::filesystem;

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

std::string PowerCap::getPcapFilename(const fs::path& path)
{
    std::regex expr{"power\\d+_cap_user$"};
    for (auto& file : fs::directory_iterator(path))
    {
        if (std::regex_search(file.path().string(), expr))
        {
            return file.path().filename();
        }
    }
    return std::string{};
}

void PowerCap::writeOcc(uint32_t pcapValue)
{
    // Create path out to master occ hwmon entry
    std::unique_ptr<fs::path> fileName =
        std::make_unique<fs::path>(OCC_HWMON_PATH);
    *fileName /= occMasterName;
    *fileName /= "/hwmon/";

    // Need to get the hwmonXX directory name, there better only be 1 dir
    assert(std::distance(fs::directory_iterator(*fileName),
                         fs::directory_iterator{}) == 1);
    // Now set our path to this full path, including this hwmonXX directory
    fileName = std::make_unique<fs::path>(*fs::directory_iterator(*fileName));
    // Append on the hwmon string where we write the user power cap

    auto baseName = getPcapFilename(*fileName);
    if (baseName.empty())
    {
        log<level::ERR>("Could not find a power cap file to write to",
                        entry("PATH=%s", *fileName->c_str()));
        return;
    }
    *fileName /= baseName;

    uint64_t microWatts = pcapValue * 1000000ull;

    auto pcapString{std::to_string(microWatts)};

    log<level::INFO>("Writing pcap value to hwmon",
                     entry("PCAP_PATH=%s", fileName->c_str()),
                     entry("PCAP_VALUE=%s", pcapString.c_str()));
    // Open the hwmon file and write the power cap
    std::ofstream file(*fileName, std::ios::out);
    file << pcapString;
    file.close();
    return;
}

void PowerCap::pcapChanged(sdbusplus::message::message& msg)
{
    if (!occStatus.occActive())
    {
        // Nothing to  do
        return;
    }

    uint32_t pcap = 0;
    bool pcapEnabled = false;

    std::string msgSensor;
    std::map<std::string, std::variant<uint32_t, bool>> msgData;
    msg.read(msgSensor, msgData);

    // Retrieve which property changed via the msg and read the other one
    auto valPropMap = msgData.find(POWER_CAP_PROP);
    if (valPropMap != msgData.end())
    {
        pcap = std::get<uint32_t>(valPropMap->second);
        pcapEnabled = getPcapEnabled();
    }
    else
    {
        valPropMap = msgData.find(POWER_CAP_ENABLE_PROP);
        if (valPropMap != msgData.end())
        {
            pcapEnabled = std::get<bool>(valPropMap->second);
            pcap = getPcap();
        }
        else
        {
            log<level::INFO>("Unknown power cap property changed");
            return;
        }
    }

    log<level::INFO>("Power Cap Property Change", entry("PCAP=%u", pcap),
                     entry("PCAP_ENABLED=%u", pcapEnabled));

    // Determine desired action to write to occ

    auto occInput = getOccInput(pcap, pcapEnabled);

    // Write action to occ
    writeOcc(occInput);

    return;
}

} // namespace powercap

} // namespace occ

} // namespace open_power
