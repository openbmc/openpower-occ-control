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
 * @param[in] objectPath - Name of the object containing the property
 * @param[in] interface - Interface name containing the property
 * @param[in] propertyName - Property name
 * @param[in] value - Property value
 */
void setProperty(const std::string& objectPath, const std::string& interface,
                 const std::string& propertyName, PropertyValue&& value)
{
    using namespace std::literals::string_literals;
    PropertyValue varValue(std::forward<PropertyValue>(value));

    try
    {
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
    catch (const std::exception& e)
    {
        auto error = errno;
        log<level::ERR>(
            fmt::format("setProperty: failed to Set {}, errno={}, what={}",
                        propertyName.c_str(), error, e.what())
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

// Get the service and object path for an interface
std::string getServiceUsingSubTree(const std::string& interface,
                                   std::string& path)
{
    using Path = std::string;
    using Intf = std::string;
    using Serv = std::string;
    using Intfs = std::vector<Intf>;
    using Objects = std::map<Path, std::map<Serv, Intfs>>;
    Serv service;
    Objects rspObjects;

    auto& bus = getBus();
    auto method = bus.new_method_call(MAPPER_BUSNAME, MAPPER_OBJ_PATH,
                                      MAPPER_IFACE, "GetSubTree");
    method.append(path, 0, std::vector{interface});

    auto mapperResponseMsg = bus.call(method);
    mapperResponseMsg.read(rspObjects);
    if (rspObjects.empty())
    {
        log<level::ERR>(
            fmt::format(
                "util::getServiceUsingSubTree: Failed getSubTree({},0,{})",
                path.c_str(), interface)
                .c_str());
    }
    else
    {
        path = rspObjects.begin()->first;
        if (!rspObjects.begin()->second.empty())
        {
            service = rspObjects.begin()->second.begin()->first;
        }
        else
        {
            log<level::ERR>(
                fmt::format(
                    "getServiceUsingSubTree: service not found for interface {} (path={})",
                    interface, path.c_str())
                    .c_str());
        }
    }

    return service;
}

} // namespace utils
} // namespace occ
} // namespace open_power
