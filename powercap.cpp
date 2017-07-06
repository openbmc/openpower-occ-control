#include <powercap.hpp>
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{
namespace powercap
{

using namespace phosphor::logging;

constexpr auto OCC_STATUS_PATH    = "/xyz/openbmc_project/occ/status";
constexpr auto OCC_STATUS_INTERFACE = "org.open_power.OCC.Status";

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

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

bool PowerCap::isOccActive()
{
    std::string occService = getService(OCC_STATUS_PATH,OCC_STATUS_INTERFACE);

    auto method = this->bus.new_method_call(occService.c_str(),
                                            OCC_STATUS_PATH,
                                            "org.freedesktop.DBus.Properties",
                                            "Get");

    method.append(OCC_STATUS_INTERFACE, "OccActive");
    auto reply = this->bus.call(method);

    if (reply.is_method_error())
    {
        log<level::ERR>("Error in OccActive Get");
        return false;
    }
    sdbusplus::message::variant<bool> occStatus;
    reply.read(occStatus);

    return sdbusplus::message::variant_ns::get<bool>(occStatus);
}

void PowerCap::updateOcc()
{

}

void PowerCap::pcapChanged(sdbusplus::message::message& msg)
{
    log<level::DEBUG>("Power Cap Change Detected");
    if(!isOccActive())
    {
        // Nothing to  do
        return;
    }

    uint32_t pcap {};
    msg.read(pcap);
    log<level::INFO>("Power Cap Changed",
                     entry("PCAP=%u",pcap));
}

void PowerCap::pcapEnableChanged(sdbusplus::message::message& msg)
{
    log<level::DEBUG>("Power Cap Enable Change Detected");
    if(!isOccActive())
    {
        // Nothing to  do
        return;
    }

    bool enabled = false;
    msg.read(enabled);
    if(enabled)
    {
        log<level::INFO>("Power Cap Enabled");
    }
    else
    {
        log<level::INFO>("Power Cap Disabled");
    }
}

} // namespace open_power

} // namespace occ

}// namespace powercap
