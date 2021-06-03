#pragma once

#include <optional>
#include <sdbusplus/bus.hpp>
#include <string>
#include <tuple>

namespace open_power
{
namespace occ
{

using LABELVALUE = std::tuple<std::string, uint16_t>;

/**
 * @brief Gets the D-Bus Service name for the input D-Bus path
 *
 * @param[in] bus  -  Bus handler
 * @param[in] path -  Object Path
 * @param[in] intf -  Interface
 *
 * @return            Service name
 * @error             InternalFailure exception thrown
 */
std::string getService(sdbusplus::bus::bus& bus, const std::string& path,
                       const std::string& intf);

/**
 * @brief Check the value of the `tempX_label` file
 *
 * @param[in] value  -  the value of the `tempX_label` file
 *
 * @return              Sensors type and Sensors ID
 */
std::optional<LABELVALUE> checkLabelValue(const std::string& value);

} // namespace occ
} // namespace open_power
