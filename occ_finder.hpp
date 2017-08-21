#pragma once

#include <vector>
#include <string>
#include <sdbusplus/bus.hpp>

namespace open_power
{
namespace occ
{
namespace finder
{

/** @brief Get OCC objects on the system by mapping them to CPU inventory
  * @returns vector of occ objects, such as occ0, occ1, and so on.
  *
  * @param[in] bus - sdbusplus handler
  */
std::vector<std::string> get(sdbusplus::bus::bus& bus);

} // namespace finder
} // namespace occ
} // namespace open_power
