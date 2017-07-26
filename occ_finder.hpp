#pragma once

#include <vector>
#include <string>

namespace open_power
{
namespace occ
{
namespace finder
{

/** @brief Get OCC objects on the system by mapping them to CPU inventory
  * @returns vector of occ objects, such as occ0, occ1, and so on.
  */
std::vector<std::string> get();

} // namespace finder
} // namespace occ
} // namespace open_power
