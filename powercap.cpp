#include <powercap.hpp>
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{
namespace powercap
{

constexpr auto PCAP_SETTINGS_SERVICE = "xyz.openbmc_project.Settings";
constexpr auto PCAP_PATH    = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";

using namespace phosphor::logging;

uint32_t PowerCap::getOccInput(uint32_t pcap, bool pcapEnabled)
{
    if(!pcapEnabled)
    {
        // Pcap disabled, return 0 to indicate disabled
        return 0;
    }

    // If pcap is not disabled then just return the pcap
    return pcap;
}

uint32_t PowerCap::getPcap()
{

    auto method = this->bus.new_method_call(PCAP_SETTINGS_SERVICE,
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

    return sdbusplus::message::variant_ns::get<uint32_t>(pcap);
}

bool PowerCap::getPcapEnabled()
{
    auto method = this->bus.new_method_call(PCAP_SETTINGS_SERVICE,
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

    return sdbusplus::message::variant_ns::get<bool>(pcapEnabled);
}

void PowerCap::writeOcc(uint32_t pcapValue)
{
    fs::path fileName {OCC_HWMON_PATH};
    fileName / "/occ1-dev0/hwmon/hwmon*/caps1_user";

    std::string data {std::to_string(pcapValue)};

    std::ofstream file(fileName, std::ios::out);
    file << data;
    file.close();
    return;
}

void PowerCap::pcapChanged(sdbusplus::message::message& msg)
{
    log<level::DEBUG>("Power Cap Setting Change Detected");
    if(!occStatus.occActive())
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
    log<level::DEBUG>("Writing new power cap setting to OCC",
                     entry("OCC_PCAP_VAL=%u",occInput));

    // Write action to occ
    writeOcc(occInput);

    return;
}

} // namespace open_power

} // namespace occ

}// namespace powercap
