#pragma once

#include <sdbusplus/bus.hpp>
#include <string>
#include <vector>

namespace open_power
{
namespace occ
{
namespace finder
{

// Map of property and interface
// This is used to filter the OCCs based on the property value
using Interface = std::string;
using Property = std::string;

using Value = bool;
using Match = std::tuple<Interface, Property, Value>;
using Criteria = std::vector<Match>;

/** @brief Get OCC objects on the system by mapping them to CPU inventory
 * @returns vector of occ objects, such as occ0, occ1, and so on.
 *
 * @param[in] bus - sdbusplus handler
 */
std::vector<std::string> get(sdbusplus::bus::bus& bus);

/** @brief Returns true if the inventory item matches the criteria
 *
 *  @param[in] bus      - sdbusplus handler
 *  @param[in] path     - D-Bus path
 *  @param[in] service  - D-Bus service name
 *  @param[in] match    - Criteria match vector
 *
 *  @return true on match, false otherwise
 */
bool matchCriteria(sdbusplus::bus::bus& bus, const std::string& path,
                   const std::string& service, const Criteria& match);

/** @brief Gets the value associated with the given object
 *         and the interface.
 *
 *  @param[in] bus       - sdbusplus handler
 *  @param[in] service   - D-Bus service name.
 *  @param[in] objPath   - D-Bus object path.
 *  @param[in] interface - D-Bus interface.
 *  @param[in] property  - Name of the property.
 *
 *  @return Value of the property
 */

template <typename T>
T getDbusProperty(sdbusplus::bus::bus& bus, const std::string& service,
                  const std::string& objPath, const std::string& interface,
                  const std::string& property);

} // namespace finder
} // namespace occ
} // namespace open_power
