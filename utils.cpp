#include "utils.hpp"

#include <systemd/sd-event.h>
#include <unistd.h>

#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/State/Boot/Progress/server.hpp>
#include <xyz/openbmc_project/State/Host/server.hpp>

#include <format>
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

using BootProgress = sdbusplus::xyz::openbmc_project::State::Boot::server::
    Progress::ProgressStages;
constexpr auto HOST_STATE_OBJ_PATH = "/xyz/openbmc_project/state/host0";

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
                std::format("util::setProperty: Failed to set property {}",
                            propertyName)
                    .c_str());
        }
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        log<level::ERR>(
            std::format("setProperty: failed to Set {}, errno={}, what={}",
                        propertyName.c_str(), error, e.what())
                .c_str());
    }
}

std::vector<std::string> getSubtreePaths(
    const std::vector<std::string>& interfaces, const std::string& path)
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
            std::format(
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
                std::format(
                    "getServiceUsingSubTree: service not found for interface {} (path={})",
                    interface, path.c_str())
                    .c_str());
        }
    }

    return service;
}

std::string getStateValue(const std::string& intf, const std::string& objPath,
                          const std::string& state)
{
    std::string stateVal;
    try
    {
        auto& bus = getBus();
        auto service = getService(objPath, intf);
        if (service.empty())
        {
            throw std::runtime_error("getStateValue: Failed to get service");
        }

        auto method =
            bus.new_method_call(service.c_str(), objPath.c_str(),
                                "org.freedesktop.DBus.Properties", "Get");

        method.append(intf, state);

        auto reply = bus.call(method);

        std::variant<std::string> propertyVal;

        reply.read(propertyVal);

        stateVal = std::get<std::string>(propertyVal);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(std::format("D-Bus call exception, OBJPATH({}), "
                                    "INTERFACE({}), PROPERTY({}) EXCEPTION({})",
                                    objPath, intf, state, e.what())
                            .c_str());
        throw std::runtime_error("Failed to get host state property");
    }
    catch (const std::bad_variant_access& e)
    {
        log<level::ERR>(
            std::format("Exception raised while read host state({}) property "
                        "value,  OBJPATH({}), INTERFACE({}), EXCEPTION({})",
                        state, objPath, intf, e.what())
                .c_str());
        throw std::runtime_error("Failed to get host state property");
    }

    return stateVal;
}

BootProgress getBootProgress()
{
    BootProgress bootProgessStage;
    constexpr auto bootProgressInterface =
        "xyz.openbmc_project.State.Boot.Progress";
    std::string value = getStateValue(bootProgressInterface,
                                      HOST_STATE_OBJ_PATH, "BootProgress");
    bootProgessStage = sdbusplus::xyz::openbmc_project::State::Boot::server::
        Progress::convertProgressStagesFromString(value);
    return bootProgessStage;
}

bool isHostRunning()
{
    BootProgress bootProgressStatus = getBootProgress();
    if ((bootProgressStatus == BootProgress::SystemInitComplete) ||
        (bootProgressStatus == BootProgress::SystemSetup) ||
        (bootProgressStatus == BootProgress::OSRunning))
    {
        return true;
    }
    return false;
}

} // namespace utils
} // namespace occ
} // namespace open_power
