#include "occ_device.hpp"
#include "occ_status.hpp"

namespace open_power
{
namespace occ
{

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

void Device::throttleTempCallback(bool error)
{
        statusObject->throttleTemp(error);
}

void Device::throttlePowerCallback(bool error)
{
        statusObject->throttlePower(error);
}

void Device::throttleMemCallback(bool error)
{
        statusObject->throttleMem(error);
}

} // namespace occ
} // namespace open_power
