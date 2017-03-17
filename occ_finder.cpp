#include <algorithm>
#include <sdbusplus/server.hpp>
#include "occ_finder.hpp"
#include "config.h"

namespace open_power
{
namespace occ
{
namespace finder
{

constexpr auto toChar(size_t c)
{
    constexpr auto map = "0123";
    return map[c];
}

std::vector<std::string> get()
{
    auto bus = sdbusplus::bus::new_default();
    auto mapper =
        bus.new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper",
            "GetSubTreePaths");

    auto depth = 0;
    mapper.append(std::string(INVENTORY_ROOT));
    mapper.append(depth);
    mapper.append(std::vector<std::string>(
        {std::string(INVENTORY_ITEM_INTERFACE)}));

    auto result = bus.call(mapper);
    if (result.is_method_error())
    {
        throw std::runtime_error("ObjectMapper GetSubTreePaths failed");
    }

    std::vector<std::string> response;
    result.read(response);
    if (response.empty())
    {
        throw std::runtime_error("ObjectMapper GetSubTreePaths : bad response");
    }

    std::vector<std::string> occs;
    size_t count = 0;
    std::string cpu = std::string(CPU_NAME) + toChar(count);
    auto constexpr MAX_PROCS = 4; // Revisit for multi-node systems
    for (const auto& path: response)
    {
        if (std::equal(cpu.crbegin(), cpu.crend(), path.crbegin()))
        {
            if(count == MAX_PROCS)
            {
                break;
            }
            occs.emplace_back(std::string(OCC_NAME) +
                              toChar(count++));
        }
        cpu.back() = toChar(count);
    }

    return occs;
}

} // namespace finder
} // namespace occ
} // namespace open_power
