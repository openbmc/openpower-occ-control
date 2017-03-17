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

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    return {};
}

} // namespace pass_through
} // namespace occ
} // namespace open_power
