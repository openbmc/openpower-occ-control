#include <memory>
#include <algorithm>
#include <phosphor-logging/log.hpp>
#include "occ_pass_through.hpp"
#include "occ_finder.hpp"

namespace open_power
{
namespace occ
{
namespace pass_through
{

void run()
{
    auto bus = sdbusplus::bus::new_default();
    sdbusplus::server::manager::manager objManager(bus,
                                                   OCC_PASS_THROUGH_ROOT);

    std::vector<std::unique_ptr<PassThrough>> objects;
    auto occs = open_power::occ::finder::get();

    for (const auto& occ : occs)
    {
        auto occPassThrough = object(occ);
        objects.emplace_back(
            std::make_unique<PassThrough>(bus, occPassThrough.c_str()));
    }
    bus.request_name(OCC_PASS_THROUGH_BUSNAME);

    while (true)
    {
        bus.process_discard();
        bus.wait();
    }
}

PassThrough::PassThrough(
    sdbusplus::bus::bus& bus,
    const char* path) :
    Iface(bus, path),
    path(path)
{
    this->emit_object_added();
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    std::string msg = "Pass through to OCC ";
    msg += path;

    std::string cmd;
    std::for_each(command.cbegin(), command.cend(),
                  [&cmd](const auto& c)
                  {
                      cmd += std::to_string(c);
                      cmd += ',';
                  });
    if (!cmd.empty())
    {
        // Remove trailing ','
        cmd.pop_back();
    }

    using namespace phosphor::logging;
    log<level::INFO>(msg.c_str(), entry("COMMAND=%s", cmd.c_str()));

    return {};
}

} // namespace pass_through
} // namespace occ
} // namespace open_power
