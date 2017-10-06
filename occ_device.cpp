#include <iostream>
#include "occ_device.hpp"

namespace open_power
{
namespace occ
{

fs::path Device::bindPath = fs::path(OCC_HWMON_PATH) / "bind";
fs::path Device::unBindPath = fs::path(OCC_HWMON_PATH) / "unbind";

bool Device::master() const
{
    int master;
    std::string masterFile(fs::path(DEV_PATH) / fs::path(config) /
                           "occ_master");
    std::ifstream file(masterFile, std::ios::in);

    if (!file)
    {
        return false;
    }

    file >> master;
    file.close();
    return master != 0;
}

} // namespace occ
} // namespace open_power
