#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace utils
{
#ifdef I2C_OCC

namespace fs = std::experimental::filesystem;

constexpr const char I2C_OCC_HWMON_DEVICE_NAME[] = "p8-occ-hwmon";
constexpr auto NAME_LENGTH = 12;
static_assert(sizeof(I2C_OCC_HWMON_DEVICE_NAME) -1 == NAME_LENGTH);

/** @brief Get file content
 *
 * Get at most NAME_LENGTH bytes of content from file. If the file is smaller
 * than NAME_LENGTH bytes, return the valid parts.
 *
 * @return The string of file content
 */
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

/** @brief Find all devices of occ hwmon
 *
 * It iterates in path, finds all occ hwmon devices
 *
 * E.g. If "path/3-0050/name" exists and its content is "p8-occ-hwmon",
 * "3-0050" is returned.
 *
 * @return A vector of strings containing the occ hwmon device path
 */
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


#endif
} // namespace utils
