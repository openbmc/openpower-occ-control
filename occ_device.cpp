#include "occ_device.hpp"

#include "occ_status.hpp"

#include <iostream>

namespace open_power
{
namespace occ
{

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

std::string Device::getPathBack(const fs::path& path)
{
    if (path.empty())
        return std::string();

    // Points to the last element in the path
    auto conf = --path.end();

    // The last element will be '.' if the path ends in '/'
    // This behavior differs between filesystem and experimental::filesystem
    // Verify there is an element before too
    if (!conf->compare(".") && conf != path.begin())
    {
        return *(--conf);
    }
    else
    {
        return *conf;
    }
}

bool Device::master() const
{
    int master;
    auto masterFile = devPath / "occ_master";
    std::ifstream file(masterFile, std::ios::in);

    if (!file)
    {
        return false;
    }

    file >> master;
    file.close();
    return (master != 0);
}

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
