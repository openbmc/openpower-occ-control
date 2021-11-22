#include "utils.hpp"

#include <fmt/core.h>

#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <string>
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

/**
 * @brief Sets a given object's property value
 *
 * @param[in] object - Name of the object containing the property
 * @param[in] interface - Interface name containing the property
 * @param[in] property - Property name
 * @param[in] value - Property value
 */
void setProperty(const std::string& objectPath, const std::string& interface,
                 const std::string& propertyName, std::string&& value)
{
    using namespace std::literals::string_literals;
    std::variant<std::string> varValue(std::forward<std::string>(value));

    auto& bus = getBus();
    auto service = getService(objectPath, interface);
    if (service.empty())
    {
        return;
    }

    auto method = bus.new_method_call(service.c_str(), objectPath.c_str(),
                                      DBUS_PROPERTY_IFACE, "Set");
    method.append(interface, propertyName, varValue);

    auto reply = bus.call(method);
    if (reply.is_method_error())
    {
        log<level::ERR>(
            fmt::format("util::setProperty: Failed to set property {}",
                        propertyName)
                .c_str());
    }
}

std::vector<std::string>
    getSubtreePaths(const std::vector<std::string>& interfaces,
                    const std::string& path)
{
    std::vector<std::string> paths;

    auto& bus = getBus();
    auto method = bus.new_method_call(MAPPER_BUSNAME, MAPPER_OBJ_PATH,
                                      MAPPER_IFACE, "GetSubTreePaths");
    method.append(path, 0, interfaces);

    auto reply = bus.call(method);
    reply.read(paths);

    return paths;
}
} // namespace utils
} // namespace occ
} // namespace open_power
