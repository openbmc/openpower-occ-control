#pragma once

#include <iostream>
#include <fstream>
#include "config.h"
namespace open_power
{
namespace occ
{

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
        Device(const std::string& path)
        {
            // Device instance number starts from 1 and
            // dbus object starts from 0
            name.append(std::to_string((path.back() - '0') + 1));

            // This is what to be written to occ device to
            // achieve bind and unbind
            config = name + '-' + "dev0";

            // Populate bind and unbind paths
            bindPath = std::string(OCC_HWMON_PATH) + "bind";
            unBindPath = std::string(OCC_HWMON_PATH) + "unbind";

            // Error file would be occ<#>-dev0/occ_error
            errorFile = std::string(OCC_HWMON_PATH) + config +
                                    '/' + "occ_error";
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
        /** @brief OCC name. Ex: occ0, occ1 etc */
        std::string name = OCC_NAME;

        /**  @brief To bind the device to the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/bind
         */
        std::string bindPath;

        /**  @brief To un-bind the device from the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/unbind
         */
        std::string unBindPath;

        /** @brief This file contains 0 for success and non-zero for failure */
        std::string errorFile;

        /** @brief Config value to be used to do bind and unbind */
        std::string config;

        /** @brief Generic file writer to achieve bind and unbind
         *
         *  @param[in] filename - Name of file to be written
         *  @param[in] data     - Data to be written to
         *  @return             - None
         */
        template <typename T>
        void write(const std::string& fileName, T&& data)
        {
            if(std::ifstream(fileName))
            {
                std::ofstream file(fileName, std::ios::out);
                file << data;
                file.close();
            }
            return;
        }
};

} // namespace occ
} // namespace open_power
