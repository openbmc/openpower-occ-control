#include <phosphor-logging/elog-errors.hpp>
#include <sdbusplus/bus.hpp>
#include <string>
#include <xyz/openbmc_project/Common/error.hpp>
namespace open_power
{
namespace occ
{

// For throwing exceptions
using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

std::string getService(sdbusplus::bus::bus& bus, const std::string& path,
                       const std::string& intf)
{
    auto mapperCall =
        bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                            "/xyz/openbmc_project/object_mapper",
                            "xyz.openbmc_project.ObjectMapper", "GetObject");

    mapperCall.append(path);
    mapperCall.append(std::vector<std::string>({intf}));

    auto mapperResponseMsg = bus.call(mapperCall);

    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>("ERROR in getting service",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", intf.c_str()));

        elog<InternalFailure>();
    }

    std::map<std::string, std::vector<std::string>> mapperResponse;
    mapperResponseMsg.read(mapperResponse);

    if (mapperResponse.begin() == mapperResponse.end())
    {
        log<level::ERR>("ERROR reading mapper response",
                        entry("PATH=%s", path.c_str()),
                        entry("INTERFACE=%s", intf.c_str()));

        elog<InternalFailure>();
    }
    return mapperResponse.begin()->first;
}

} // namespace occ
} // namespace open_power
