#include <algorithm>
#include <fstream>

#include "utils.hpp"

namespace utils
{
#ifdef I2C_OCC

namespace fs = std::experimental::filesystem;

constexpr const char I2C_OCC_HWMON_DEVICE_NAME[] = "p8-occ-hwmon";
constexpr auto NAME_LENGTH = 12;
static_assert(sizeof(I2C_OCC_HWMON_DEVICE_NAME) -1 == NAME_LENGTH);

std::string getFileContent(const fs::path& f)
{
    std::string ret(NAME_LENGTH, 0);
    std::ifstream ifs(f.c_str(), std::ios::binary);
    if (ifs.is_open())
    {
        ifs.read(&ret[0], NAME_LENGTH);
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
            if (str == I2C_OCC_HWMON_DEVICE_NAME)
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
    dbusToI2c(name);
    return name;
}

#endif
} // namespace utils
