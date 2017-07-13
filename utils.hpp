#pragma once

#include <experimental/filesystem>
#include <sdbusplus/bus.hpp>
#include <string>
#include <vector>

namespace open_power
{
namespace occ
{
/**
 * @brief Gets the D-Bus Service name for the input D-Bus path
 *
 * @param[in] bus  -  Bus handler
 * @param[in] intf -  Interface
 * @param[in] path -  Object Path
 *
 * @return            Service name
 * @error             InternalFailure exception thrown
 */
std::string getService(sdbusplus::bus::bus& bus,
                       const std::string& intf,
                       const std::string& path);
} // namespace occ
} // namespace open_power

namespace utils
{
#ifdef I2C_OCC

namespace fs = std::experimental::filesystem;

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

#endif
} // namespace utils
