#pragma once

#include <sdbusplus/bus.hpp>
#include <string>

namespace open_power
{
namespace occ
{
/**
 * @brief Gets the D-Bus Service name for the input D-Bus path
 *
 * @param[in] bus  -  Bus handler
 * @param[in] intf -  Interface
 * @param[in] path -  Object Path
 *
 * @return            Service name
 * @error             InternalFailure exception thrown
 */
std::string getService(sdbusplus::bus::bus& bus,
                       const std::string& intf,
                       const std::string& path);
} // namespace occ
} // namespace open_power

