#include "utils.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/bus.hpp>
#include <string>
#include <xyz/openbmc_project/Common/error.hpp>
namespace open_power
{
namespace occ
{

// For throwing exceptions
using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

std::string getService(sdbusplus::bus::bus& bus, const std::string& path,
                       const std::string& intf)
{
    auto mapperCall =
        bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetObject");

    mapperCall.append(path);
    mapperCall.append(std::vector<std::string>({intf}));

    auto mapperResponseMsg = bus.call(mapperCall);

    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>("ERROR in getting service",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", intf.c_str()));

        elog<InternalFailure>();
    }

    std::map<std::string, std::vector<std::string>> mapperResponse;
    mapperResponseMsg.read(mapperResponse);

    if (mapperResponse.begin() == mapperResponse.end())
    {
        log<level::ERR>("ERROR reading mapper response",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", intf.c_str()));

        elog<InternalFailure>();
    }
    return mapperResponse.begin()->first;
}

std::optional<LABLEVALUE> checkLabelValue(const std::string& value)
{
    // eg: Dimm2, the lable file is `D0000002`
    // so value length = 2 byte(type) + 2 byte(reserve) + 4 byte(instace ID)
    size_t valueLen = value.length();
    size_t typeLen = 2;
    size_t reserveLen = 2;
    size_t instaceIDLen = 4;
    if (valueLen != typeLen + reserveLen + instaceIDLen)
    {
        return std::nullopt;
    }

    size_t offset = 0;
    std::string type = value.substr(offset, typeLen);
    offset += typeLen;
    std::string reserve = value.substr(offset, reserveLen);
    offset += reserveLen;

    if ("00" != reserve)
    {
        return std::nullopt;
    }

    const char* start = value.data() + offset;
    uint32_t instaceID = (uint32_t)std::strtol(start, NULL, 16);

    LABLEVALUE labelValue{type, instaceID};

    return std::make_optional(std::move(labelValue));
}

} // namespace occ
} // namespace open_power
