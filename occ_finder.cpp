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

std::vector<std::string> get()
{
    namespace fs = std::experimental::filesystem;
    using Path = std::string;
    using Service = std::string;
    using Interface = std::string;
    using Interfaces = std::vector<Interface>;

    auto bus = sdbusplus::bus::new_default();
    auto mapper =
        bus.new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper",
            "GetSubTree");

    auto depth = 0;
    Path path = CPU_SUBPATH;
    Interfaces interfaces{INVENTORY_ITEM_INTERFACE};
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
    for (auto count = 0; count < MAX_CPUS; ++count)
    {
        fs::path p(path);
        p /= std::string(CPU_NAME) + toChar(count);
        if (response.end() != response.find(p.string()))
        {
            occs.emplace_back(std::string(OCC_NAME) +
                              toChar(count));
        }
    }

    return occs;
}

} // namespace finder
} // namespace occ
} // namespace open_power
