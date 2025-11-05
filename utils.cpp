#include "utils.hpp"

#include <systemd/sd-event.h>
#include <unistd.h>

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/State/Boot/Progress/server.hpp>
#include <xyz/openbmc_project/State/Host/server.hpp>

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
        lg2::error("ERROR reading mapper response: path={PATH}, I/F={INTF}",
                   "PATH", path, "INTF", interface);

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

        bus.call(method);
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        lg2::error("setProperty: failed to Set {PROP}, errno={ERR}, what={MSG}",
                   "PROP", propertyName, "ERR", error, "MSG", e);
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
        lg2::error(
            "util::getServiceUsingSubTree: Failed getSubTree({PATH},0,{INTF})",
            "PATH", path, "INTF", interface);
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
            lg2::error(
                "getServiceUsingSubTree: service not found for interface {INTF} (path={PATH})",
                "INTF", interface, "PATH", path);
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

        auto propertyVal = reply.unpack<std::variant<std::string>>();

        stateVal = std::get<std::string>(propertyVal);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("D-Bus call exception, OBJPATH({PATH}), "
                   "INTERFACE({INTF}), PROPERTY({PROP}) EXCEPTION({ERR})",
                   "PATH", objPath, "INTF", intf, "PROP", state, "ERR",
                   e.what());
        throw std::runtime_error("Failed to get host state property");
    }
    catch (const std::bad_variant_access& e)
    {
        lg2::error(
            "Exception raised while read host state({STATE}) property "
            "value,  OBJPATH({PATH}), INTERFACE({INTF}), EXCEPTION({ERR})",
            "STATE", state, "PATH", objPath, "INTF", intf, "ERR", e.what());
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

// Convert vector to hex dump string
std::vector<std::string> hex_dump(const std::vector<std::uint8_t>& data,
                                  const unsigned int data_len)
{
    unsigned int dump_length = data.size();
    if ((data_len > 0) && (data_len < dump_length))
    {
        dump_length = data_len;
    }
    std::vector<std::string> dumpString;
    std::string s;
    for (uint32_t i = 0; i < dump_length; i++)
    {
        if (i % 16 == 0)
        {
            s += std::format("{:04X}: ", i);
        }
        else if (i % 4 == 0)
        {
            s += " ";
        }

        s += std::format("{:02X}", data.at(i));

        if ((i % 16 == 15) || (i == (dump_length - 1)))
        {
            dumpString.push_back(s);
            s.clear();
        }
    }
    return dumpString;
}

} // namespace utils
} // namespace occ
} // namespace open_power
