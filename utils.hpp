#pragma once

#include <optional>
#include <sdbusplus/bus.hpp>
#include <string>
#include <tuple>

namespace open_power
{
namespace occ
{
namespace utils
{

using LABELVALUE = std::tuple<std::string, uint16_t>;

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_OBJ_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_IFACE = "xyz.openbmc_project.ObjectMapper";
constexpr auto DBUS_PROPERTY_IFACE = "org.freedesktop.DBus.Properties";

// The value of the property(type: variant, contains some basic types)
using PropertyValue = std::variant<uint32_t, bool>;

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
 *  @throw sdbusplus::exception::SdBusError when it fails
 */
const PropertyValue getProperty(const std::string& objectPath,
                                const std::string& interface,
                                const std::string& propertyName);

/**
 * @brief Check the value of the `tempX_label` file
 *
 * @param[in] value  -  the value of the `tempX_label` file
 *
 * @return              Sensors type and Sensors ID
 */
std::optional<LABELVALUE> checkLabelValue(const std::string& value);

} // namespace utils
} // namespace occ
} // namespace open_power
