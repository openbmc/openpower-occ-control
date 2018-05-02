#include "occ_device.hpp"
#include "occ_bus.hpp"

namespace open_power
{
namespace occ
{

fs::path Bus::bindPath = fs::path(OCC_BUS_PATH) / "bind";
fs::path Bus::unBindPath = fs::path(OCC_BUS_PATH) / "unbind";

void Bus::reset() const
{
#ifndef I2C_OCC
    Device::write(unBindPath, config);
    Device::write(bindPath, config);
#endif
}

} // namespace occ
} // namespace open_power
