#include "config.h"

#include "i2c_occ.hpp"

#include <algorithm>
#include <cassert>
#include <fstream>

#ifdef I2C_OCC

namespace i2c_occ
{

namespace fs = std::filesystem;

// The occ_master sysfs file
constexpr auto OCC_MASTER_FILE = "occ_master";
// The device name's length, e.g. "p8-occ-hwmon"
constexpr auto DEVICE_NAME_LENGTH = 12;
// The occ name's length, e.g. "occ"
constexpr auto OCC_NAME_LENGTH = 3;

// static assert to make sure the i2c occ device name is expected
static_assert(sizeof(I2C_OCC_DEVICE_NAME) - 1 == DEVICE_NAME_LENGTH);
static_assert(sizeof(OCC_NAME) - 1 == OCC_NAME_LENGTH);

static bool isMasterOcc(const fs::directory_entry& p)
{
    auto f = p / fs::path{OCC_MASTER_FILE};
    auto str = getFileContent(f);
    return (!str.empty()) && (str[0] == '1');
}

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
        for (auto& p : fs::directory_iterator(path))
        {
            // Check if a device's name is "p8-occ-hwmon"
            auto f = p / fs::path{"name"};
            auto str = getFileContent(f);
            if (str == I2C_OCC_DEVICE_NAME)
            {
                if (isMasterOcc(p))
                {
                    // Insert master occ at the beginning
                    result.emplace(result.begin(), p.path().filename());
                }
                else
                {
                    result.emplace_back(p.path().filename());
                }
            }
        }
    }
    if (!result.empty())
    {
        // Sort the occ devices except for master
        std::sort(result.begin() + 1, result.end());
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

#endif // I2C_OCC
