#include "occ_device.hpp"
#include "occ_status.hpp"

namespace open_power
{
namespace occ
{

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

void Device::throttleProcTempCallback(bool error)
{
        statusObject.throttleProcTemp(error);
}

void Device::throttleProcPowerCallback(bool error)
{
        statusObject.throttleProcPower(error);
}

void Device::throttleMemTempCallback(bool error)
{
        statusObject.throttleMemTemp(error);
}

} // namespace occ
} // namespace open_power
