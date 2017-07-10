#include <powercap.hpp>
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{
namespace powercap
{

constexpr auto PCAP_PATH    = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";

using namespace phosphor::logging;

std::string PowerCap::getService(std::string path,
                                 std::string interface)
{
    auto mapper = bus.new_method_call(MAPPER_BUSNAME,
                                      MAPPER_PATH,
                                      MAPPER_INTERFACE,
                                      "GetObject");

    mapper.append(path, std::vector<std::string>({interface}));
    auto mapperResponseMsg = bus.call(mapper);

    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>("Error in mapper call",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));
        // TODO openbmc/openbmc#851 - Once available, throw returned error
        throw std::runtime_error("Error in mapper call");
    }

    std::map<std::string, std::vector<std::string>> mapperResponse;
    mapperResponseMsg.read(mapperResponse);
    if (mapperResponse.empty())
    {
        log<level::ERR>("Error reading mapper response",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));
        // TODO openbmc/openbmc#1712 - Handle empty mapper resp. consistently
        throw std::runtime_error("Error reading mapper response");
    }

    return mapperResponse.begin()->first;
}

uint32_t PowerCap::getOccInput(uint32_t pcap, bool pcapEnabled)
{
    if(!pcapEnabled)
    {
        // Pcap disabled, return 0 to indicate disabled
        return 0;
    }

    // If pcap is not disabled then just return the pcap with the derating
    // factor applied.
    return (( static_cast<uint64_t>(pcap) *
            PS_DERATING_FACTOR)/100 );
}

uint32_t PowerCap::getPcap()
{
    auto settingService = getService(PCAP_PATH,PCAP_INTERFACE);

    auto method = this->bus.new_method_call(settingService.c_str(),
                                            PCAP_PATH,
                                            "org.freedesktop.DBus.Properties",
                                            "Get");

    method.append(PCAP_INTERFACE, POWER_CAP_PROP);
    auto reply = this->bus.call(method);

    if (reply.is_method_error())
    {
        log<level::ERR>("Error in getPcap prop");
        return 0;
    }
    sdbusplus::message::variant<uint32_t> pcap;
    reply.read(pcap);

    return pcap.get<uint32_t>();
}

bool PowerCap::getPcapEnabled()
{
    auto settingService = getService(PCAP_PATH,PCAP_INTERFACE);

    auto method = this->bus.new_method_call(settingService.c_str(),
                                            PCAP_PATH,
                                            "org.freedesktop.DBus.Properties",
                                            "Get");

    method.append(PCAP_INTERFACE, POWER_CAP_ENABLE_PROP);
    auto reply = this->bus.call(method);

    if (reply.is_method_error())
    {
        log<level::ERR>("Error in getPcapEnabled prop");
        return 0;
    }
    sdbusplus::message::variant<bool> pcapEnabled;
    reply.read(pcapEnabled);

    return pcapEnabled.get<bool>();
}

void PowerCap::writeOcc(uint32_t pcapValue)
{
    // Create path out to master occ hwmon entry
    std::unique_ptr<fs::path> fileName =
            std::make_unique<fs::path>(OCC_HWMON_PATH);
    *fileName /= OCC_MASTER_NAME;
    *fileName /= "/hwmon/";

    // Need to get the hwmonXX directory name, there better only be 1 dir
    assert(std::distance(fs::directory_iterator(*fileName),
                         fs::directory_iterator{}) == 1);
    // Now set our path to this full path, including this hwmonXX directory
    fileName = std::make_unique<fs::path>(*fs::directory_iterator(*fileName));
    // Append on the hwmon string where we write the user power cap
    *fileName /= "/caps1_user";

    auto pcapString {std::to_string(pcapValue)};

    log<level::INFO>("Writing pcap value to hwmon",
                     entry("PCAP_PATH=%s",*fileName),
                     entry("PCAP_VALUE=%s",pcapString.c_str()));
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
    std::map<std::string, sdbusplus::message::variant<uint32_t, bool>> msgData;
    msg.read(msgSensor, msgData);

    // Retrieve which property changed via the msg and read the other one
    auto valPropMap = msgData.find(POWER_CAP_PROP);
    if (valPropMap != msgData.end())
    {
        pcap = sdbusplus::message::variant_ns::get<uint32_t>(
            valPropMap->second);
        pcapEnabled = getPcapEnabled();
    }
    else
    {
        valPropMap = msgData.find(POWER_CAP_ENABLE_PROP);
        if (valPropMap != msgData.end())
        {
            pcapEnabled = sdbusplus::message::variant_ns::get<bool>(
                valPropMap->second);
            pcap = getPcap();
        }
        else
        {
            log<level::INFO>("Unknown power cap property changed");
            return;
        }
    }

    log<level::INFO>("Power Cap Property Change",
                     entry("PCAP=%u",pcap),
                     entry("PCAP_ENABLED=%u",pcapEnabled));

    // Determine desired action to write to occ
    uint32_t occInput = getOccInput(pcap, pcapEnabled);

    // Write action to occ
    writeOcc(occInput);

    return;
}

} // namespace open_power

} // namespace occ

}// namespace powercap
