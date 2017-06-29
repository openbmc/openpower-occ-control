#pragma once

#include <fstream>
#include <experimental/filesystem>
#include "occ_events.hpp"
#include "occ_errors.hpp"
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
         *  @param[in] event    - Unique ptr reference to sd_event
         *  @param[in] name     - OCC instance name
         *  @param[in] callback - Optional callback on errors
         */
        Device(EventPtr& event,
               const std::string& name,
               std::function<void()> callBack = nullptr) :
            config(name + '-' + "dev0"),
            errorFile(fs::path(config) / "occ_error"),
            bindPath(fs::path(OCC_HWMON_PATH) / "bind"),
            unBindPath(fs::path(OCC_HWMON_PATH) / "unbind"),
            error(event, errorFile, callBack)
        {
            // Nothing to do here
        }

        /** @brief Binds device to the OCC driver */
        inline void bind()
        {
            // Bind the device
            return write(bindPath, config);
        }

        /** @brief Un-binds device from the OCC driver */
        inline void unBind()
        {
           // Unbind the device
           return write(unBindPath, config);
        }

        /** @brief Starts to monitor for errors */
        inline void addErrorWatch()
        {
            return error.addWatch();
        }

        /** @brief stops monitoring for errors */
        inline void removeErrorWatch()
        {
           return error.removeWatch();
        }

    private:
        /** @brief Config value to be used to do bind and unbind */
        std::string config;

        /** @brief This file contains 0 for success, non-zero for errors */
        fs::path errorFile;

        /**  @brief To bind the device to the OCC driver, do:
         *
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/bind
         */
        fs::path bindPath;

        /**  @brief To un-bind the device from the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/unbind
         */
        fs::path unBindPath;

        /** Abstraction of error monitoring */
        Error error;

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
