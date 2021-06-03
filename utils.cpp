#include "utils.hpp"

#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/bus.hpp>
#include <string>
#include <xyz/openbmc_project/Common/error.hpp>
namespace open_power
{
namespace occ
{
namespace utils
{
// For throwing exceptions
using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

const std::string getService(const std::string& path,
                             const std::string& interface)
{

    using InterfaceList = std::vector<std::string>;
    std::map<std::string, std::vector<std::string>> mapperResponse;

    auto& bus = getBus();

    auto mapper = bus.new_method_call(MAPPER_BUSNAME, MAPPER_OBJ_PATH,
                                      MAPPER_IFACE, "GetObject");
    mapper.append(path, InterfaceList({interface}));

    auto mapperResponseMsg = bus.call(mapper);
    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>("ERROR in getting service",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));

        elog<InternalFailure>();
    }

    mapperResponseMsg.read(mapperResponse);
    if (mapperResponse.empty())
    {
        log<level::ERR>("ERROR reading mapper response",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));

        elog<InternalFailure>();
    }

    // the value here will be the service name
    return mapperResponse.cbegin()->first;
}

const PropertyValue getProperty(const std::string& objectPath,
                                const std::string& interface,
                                const std::string& propertyName)
{
    PropertyValue value{};

    auto& bus = getBus();
    auto service = getService(objectPath, interface);
    if (service.empty())
    {
        return value;
    }

    auto method = bus.new_method_call(service.c_str(), objectPath.c_str(),
                                      DBUS_PROPERTY_IFACE, "Get");
    method.append(interface, propertyName);

    auto reply = bus.call(method);
    reply.read(value);

    return value;
}

std::optional<LABELVALUE> checkLabelValue(const std::string& value)
{
    // The ID 0xD0000002 is only 4 bytes long, but when converted to a string it
    // is 8 characters long
    // eg: Dimm2, the lable file is `D0000002` so value
    // length = 2 byte(type) + 2 byte(reserve) + 4 byte(instace ID)
    size_t valueLen = value.length();
    size_t typeLen = 2;
    size_t reserveLen = 2;
    size_t instanceIDLen = 4;
    if (valueLen != typeLen + reserveLen + instanceIDLen)
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
    uint16_t instanceID = static_cast<uint16_t>(std::strtol(start, NULL, 16));

    LABELVALUE labelValue{type, instanceID};

    return std::make_optional(std::move(labelValue));
}

} // namespace utils
} // namespace occ
} // namespace open_power
