#include <algorithm>
#include <cassert>
#include <fstream>

#include "config.h"
#include "i2c_occ.hpp"

#ifdef I2C_OCC

namespace i2c_occ
{

namespace fs = std::experimental::filesystem;

// The device name's length, e.g. "p8-occ-hwmon"
constexpr auto DEVICE_NAME_LENGTH = 12;
// The occ name's length, e.g. "occ"
constexpr auto OCC_NAME_LENGTH = 3;

// static assert to make sure the i2c occ device name is expected
static_assert(sizeof(I2C_OCC_DEVICE_NAME) -1 == DEVICE_NAME_LENGTH);
static_assert(sizeof(OCC_NAME) -1 == OCC_NAME_LENGTH);

std::string getFileContent(const fs::path& f)
{
    std::string ret(DEVICE_NAME_LENGTH, 0);
    std::ifstream ifs(f.c_str(), std::ios::binary);
    if (ifs.is_open())
    {
        ifs.read(&ret[0], DEVICE_NAME_LENGTH);
        ret.resize(ifs.gcount());
    }
    return ret;
}

std::vector<std::string> getOccHwmonDevices(const char* path)
{
    std::vector<std::string> result{};

    if (fs::is_directory(path))
    {
        for (auto & p : fs::directory_iterator(path))
        {
            // Check if a device's name is "p8-occ-hwmon"
            auto f = p / "name";
            auto str = getFileContent(f);
            if (str == I2C_OCC_DEVICE_NAME)
            {
                result.emplace_back(p.path().filename());
            }
        }
        std::sort(result.begin(), result.end());
    }
    return result;
}

void i2cToDbus(std::string& path)
{
    std::replace(path.begin(), path.end(), '-', '_');
}

void dbusToI2c(std::string& path)
{
    std::replace(path.begin(), path.end(), '_', '-');
}

std::string getI2cDeviceName(const std::string& dbusPath)
{
    auto name = fs::path(dbusPath).filename().string();

    // Need to make sure the name starts with "occ"
    assert(name.compare(0, OCC_NAME_LENGTH, OCC_NAME) == 0);

    // Change name like occ_3_0050 to 3_0050
    name.erase(0, OCC_NAME_LENGTH + 1);

    dbusToI2c(name);
    return name;
}

} // namespace i2c_occ

#endif

