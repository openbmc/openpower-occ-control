#include "occ_pass_through.hpp"

namespace open_power
{
namespace occ
{
namespace pass_through
{

PassThrough::PassThrough(
    sdbusplus::bus::bus& bus,
    const char* path) :
    Iface(bus, path),
    path(path)
{
    this->emit_object_added();
}

std::vector<std::string> PassThrough::send(std::vector<std::string> command)
{
    return {};
}

} // namespace pass_through
} // namespace occ
} // namespace open_power
