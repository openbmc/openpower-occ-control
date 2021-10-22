#pragma once

#include <sdbusplus/bus.hpp>

#include <optional>
#include <string>
#include <tuple>

namespace open_power
{
namespace occ
{
namespace utils
{

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_OBJ_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_IFACE = "xyz.openbmc_project.ObjectMapper";
constexpr auto DBUS_PROPERTY_IFACE = "org.freedesktop.DBus.Properties";

// The value of the property(type: variant, contains some basic types)
using PropertyValue = std::variant<uint32_t, bool, double>;

/** @brief Get the bus connection. */
static auto& getBus()
{
    static auto bus = sdbusplus::bus::new_default();
    return bus;
}

/**
 *  @brief Get service name by the path and interface of the DBus.
 *
 *  @param[in] path      -  D-Bus object path
 *  @param[in] interface -  D-Bus Interface
 *
 *  @return std::string  -  the D-Bus service name
 *
 */
const std::string getService(const std::string& path,
                             const std::string& interface);

/** @brief Get property(type: variant)
 *
 *  @param[in] objectPath       -   D-Bus object path
 *  @param[in] interface        -   D-Bus interface
 *  @param[in] propertyName     -   D-Bus property name
 *
 *  @return The value of the property(type: variant)
 *
 *  @throw sdbusplus::exception::exception when it fails
 */
const PropertyValue getProperty(const std::string& objectPath,
                                const std::string& interface,
                                const std::string& propertyName);

/** @brief Get subtree paths
 *
 *  @param[in] interfaces -   D-Bus interfaces
 *  @param[in] path       -   D-Bus object path
 *
 *  @return The D-Bus paths from the GetSubTree method
 *
 *  @throw sdbusplus::exception::exception when it fails
 */
std::vector<std::string>
    getSubtreePaths(const std::vector<std::string>& interfaces,
                    const std::string& path = "/");

} // namespace utils
} // namespace occ
} // namespace open_power
