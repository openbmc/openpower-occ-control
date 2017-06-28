#pragma once

#include <fstream>
#include <experimental/filesystem>
#include "config.h"
namespace open_power
{
namespace occ
{

namespace fs = std::experimental::filesystem;

/** @class Device
 *  @brief Binds and unbinds the OCC driver upon request
 */
class Device
{
    public:
        Device() = delete;
        ~Device() = default;
        Device(const Device&) = delete;
        Device& operator=(const Device&) = delete;
        Device(Device&&) = default;
        Device& operator=(Device&&) = default;

        /** @brief Constructs the Device object
         *
         *  @param[in] name - OCC instance name
         */
        Device(const std::string& name) :
            config(name + '-' + "dev0"),
            bindPath(fs::path(OCC_HWMON_PATH) / "bind"),
            unBindPath(fs::path(OCC_HWMON_PATH) / "unbind")
        {
            // Nothing to do here
        }

        /** @brief Binds device to the OCC driver */
        inline void bind()
        {
            return write(bindPath, config);
        }

        /** @brief Un-binds device from the OCC driver */
        inline void unBind()
        {
            return write(unBindPath, config);
        }

    private:
        /** @brief Config value to be used to do bind and unbind */
        std::string config;

        /**  @brief To bind the device to the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/bind
         */
        fs::path bindPath;

        /**  @brief To un-bind the device from the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/unbind
         */
        fs::path unBindPath;

        /** @brief Generic file writer to achieve bind and unbind
         *
         *  @param[in] filename - Name of file to be written
         *  @param[in] data     - Data to be written to
         *  @return             - None
         */
        template <typename T>
        void write(const fs::path& fileName, T&& data)
        {
            if(std::ifstream(fileName))
            {
                std::ofstream file(fileName, std::ios::out);
                file << std::forward<T>(data);
                file.close();
            }
            return;
        }
};

} // namespace occ
} // namespace open_power
