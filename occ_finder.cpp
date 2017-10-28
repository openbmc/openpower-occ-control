#include <cstring>
#include <algorithm>
#include <iterator>
#include <experimental/filesystem>
#include <sdbusplus/server.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include "occ_finder.hpp"
#include "config.h"
namespace open_power
{
namespace occ
{
namespace finder
{

using namespace phosphor::logging;

constexpr auto toChar(size_t c)
{
    constexpr auto map = "0123456789abcdef";
    return map[c];
}

template <typename T>
T getDbusProperty(sdbusplus::bus::bus& bus,
                  const std::string& service,
                  const std::string& objPath,
                  const std::string& interface,
                  const std::string& property)
{
    using namespace sdbusplus::xyz::openbmc_project::Common::Error;

    constexpr auto PROPERTY_INTF = "org.freedesktop.DBus.Properties";

    auto method = bus.new_method_call(
                      service.c_str(),
                      objPath.c_str(),
                      PROPERTY_INTF,
                      "Get");
    method.append(interface, property);

    auto reply = bus.call(method);
    if (reply.is_method_error())
    {
         log<level::ERR>("Failed to get property",
                        entry("PROPERTY=%s", property.c_str()),
                        entry("PATH=%s", objPath.c_str()),
                        entry("INTERFACE=%s", interface.c_str()));
        elog<InternalFailure>();
    }

    sdbusplus::message::variant<T> value;
    reply.read(value);

    return sdbusplus::message::variant_ns::get<T>(value);
}

std::vector<std::string> get(sdbusplus::bus::bus& bus)
{
    namespace fs = std::experimental::filesystem;
    using Path = std::string;
    using Service = std::string;
    using Interface = std::string;
    using Interfaces = std::vector<Interface>;

    auto mapper =
        bus.new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper",
            "GetSubTree");

    auto depth = 0;
    Path path = CPU_SUBPATH;
    Interfaces interfaces
    {
        "xyz.openbmc_project.Inventory.Item.Cpu",
    };

    mapper.append(path);
    mapper.append(depth);
    mapper.append(interfaces);

    auto result = bus.call(mapper);
    if (result.is_method_error())
    {
        // It's okay to not have inventory, for example at BMC standby
        return {};
    }

    using MapperResponse = std::map<Path, std::map<Service, Interfaces>>;
    MapperResponse response;
    result.read(response);
    if (response.empty())
    {
        // It's okay to not have inventory, for example at BMC standby
        return {};
    }

    std::vector<std::string> occs;
    for (const auto& iter: response)
    {
        Criteria match{};
        match.emplace_back(std::make_tuple(
                           "xyz.openbmc_project.Inventory.Item",
                           "Present",
                           true));

        // Select only if the CPU is marked 'Present'.
        // Local variable to make it readable
        auto path = iter.first;
        auto service = iter.second.begin()->first;
        if (matchCriteria(bus, path, service, match))
        {
            auto index = path.find(CPU_NAME);
            std::string occ(OCC_NAME);
            occ.append(path.substr(index + std::strlen(CPU_NAME)));
            occs.emplace_back(occ);
        }
    }

    return occs;
}

bool matchCriteria(sdbusplus::bus::bus& bus,
                   const std::string& path,
                   const std::string& service,
                   const Criteria& match)
{
    for (const auto& iter: match)
    {
        auto result = getDbusProperty<bool>(bus, service, path,
                                            std::get<0>(iter),
                                            std::get<1>(iter));
        if (result != std::get<2>(iter))
        {
            return false;
        }
    }
    return true;
}

} // namespace finder
} // namespace occ
} // namespace open_power
