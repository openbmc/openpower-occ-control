#include <memory>
#include <algorithm>
#include <fcntl.h>
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
    path(path),
    fd(openDevice())
{
    // Nothing to do.
}

int PassThrough::openDevice()
{
    // Device instance number starts from 1.
    devicePath.append(std::to_string((this->path.back() - '0') + 1));

    int fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        // This is for completion. This is getting replaced by elog
        // in the next commit
        throw std::runtime_error("Error opening " + devicePath);
    }
    return fd;
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    return {};
}

} // namespace pass_through
} // namespace occ
} // namespace open_power
