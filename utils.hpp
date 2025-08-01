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
using PropertyValue =
    std::variant<uint32_t, bool, double, std::string, std::vector<std::string>>;

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
 *  @throw sdbusplus::exception_t when it fails
 */
const PropertyValue getProperty(const std::string& objectPath,
                                const std::string& interface,
                                const std::string& propertyName);

/**
 * @brief Sets a given object's property value
 *
 * @param[in] object - Name of the object containing the property
 * @param[in] interface - Interface name containing the property
 * @param[in] property - Property name
 * @param[in] value - Property value
 */
void setProperty(const std::string& objectPath, const std::string& interface,
                 const std::string& propertyName, PropertyValue&& value);

/** @brief Get subtree paths
 *
 *  @param[in] interfaces -   D-Bus interfaces
 *  @param[in] path       -   D-Bus object path
 *
 *  @return The D-Bus paths from the GetSubTree method
 *
 *  @throw sdbusplus::exception_t when it fails
 */
std::vector<std::string> getSubtreePaths(
    const std::vector<std::string>& interfaces, const std::string& path = "/");

/**
 * @brief Get the D-Bus service and object path for an interface
 *
 * @param[in] interface - D-Bus interface name
 * @param[in,out] path  - D-Bus object path
 *
 * @return D-Bus service name
 */
std::string getServiceUsingSubTree(const std::string& interface,
                                   std::string& path);

/**
 * @brief Get status of the host
 *
 * @return true is the host is running, else false
 */
bool isHostRunning();

/**
 * @brief Convert vector to hex dump string
 *
 * @param[in] data       - vector of uint8_ data
 * @param[in] path       - length of data to use (0 = all data)
 *
 * @return vector of strings
 */
std::vector<std::string> hex_dump(const std::vector<std::uint8_t>& data,
                                  const unsigned int data_len = 0);

} // namespace utils
} // namespace occ
} // namespace open_power
