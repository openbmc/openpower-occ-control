#pragma once

#include <filesystem>
#include <vector>

#ifdef I2C_OCC

namespace i2c_occ
{

namespace fs = std::filesystem;

/** @brief Get file content
 *
 * Get at most NAME_LENGTH bytes of content from file. If the file is smaller
 * than NAME_LENGTH bytes, return the valid parts.
 *
 * @param[in] f - The path of file
 *
 * @return The string of file content
 */
std::string getFileContent(const fs::path& f);

/** @brief Find all devices of occ hwmon
 *
 * It iterates in path, finds all occ hwmon devices
 *
 * E.g. If "path/3-0050/name" exists and its content is "p8-occ-hwmon",
 * "3-0050" is returned.
 *
 * @param[in] path - The path to search
 *
 * @return A vector of strings containing the occ hwmon device path
           where the first device is master occ
 */
std::vector<std::string> getOccHwmonDevices(const char* path);

/** @brief Convert i2c name to DBus path
 *
 * It converts '-' to '_' so that it becomes a valid DBus path.
 * E.g. 3-0050 converts to 3_0050
 *
 * @param[in,out] path - The i2c name to convert
 */
void i2cToDbus(std::string& name);

/** @brief Convert DBus path to i2c name
 *
 * It converts '_' to '_' so that it becomes a valid i2c name
 * E.g. 3_0050 converts to 3-0050
 *
 * @param[in,out] path - The DBus path to convert
 */
void dbusToI2c(std::string& path);

/** @brief Get i2c name from full DBus path
 *
 * It extract the i2c name from the full DBus path.
 * E.g. /org/open_power/control/3_0050 returns "3-0050"
 *
 * @param[in] dbusPath - The full DBus path
 *
 * @return The i2c name
 */
std::string getI2cDeviceName(const std::string& dbusPath);

} // namespace i2c_occ

#endif // I2C_OCC
