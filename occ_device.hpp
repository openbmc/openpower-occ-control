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

class Status;
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
               Status *status,
               std::function<void(bool)> callBack = nullptr) :
#ifdef I2C_OCC
            config(name),
#else
            config(name + '-' + "dev0"),
#endif
            errorFile(fs::path(config) / "occ_error"),
            statusObject(status),
            error(event, errorFile, callBack),
            throttleTemp(event, fs::path(config) / "occ_dvfs_ot",
                         std::bind(std::mem_fn(&Device::throttleTempCallback),
                                   this, std::placeholders::_1)),
            throttlePower(event, fs::path(config) / "occ_dvfs_power",
                          std::bind(std::mem_fn(&Device::throttlePowerCallback),
                                    this, std::placeholders::_1)),
            throttleMem(event, fs::path(config) / "occ_mem_throttle",
                        std::bind(std::mem_fn(&Device::throttleMemCallback),
                                  this, std::placeholders::_1))
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

        /** @brief Returns if device is already bound.
         *
         *  On device bind, a soft link by the name $config
         *  gets created in OCC_HWMON_PATH and gets removed
         *  on unbind
         *
         *  @return true if bound, else false
         */
        inline bool bound() const
        {
            return fs::exists(OCC_HWMON_PATH + config);
        }

        /** @brief Starts to monitor for errors */
        inline void addErrorWatch()
        {
            throttleTemp.addWatch();
            throttlePower.addWatch();
            throttleMem.addWatch();
            error.addWatch();
        }

        /** @brief stops monitoring for errors */
        inline void removeErrorWatch()
        {
            error.removeWatch();
            throttleMem.removeWatch();
            throttlePower.removeWatch();
            throttleTemp.removeWatch();
        }

    private:
        /** @brief Config value to be used to do bind and unbind */
        const std::string config;

        /** @brief This file contains 0 for success, non-zero for errors */
        const fs::path errorFile;

        /**  @brief To bind the device to the OCC driver, do:
         *
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/bind
         */
        static fs::path bindPath;

        /**  @brief To un-bind the device from the OCC driver, do:
         *    Write occ<#>-dev0 to: /sys/bus/platform/drivers/occ-hwmon/unbind
         */
        static fs::path unBindPath;

        /**  Store the associated Status instance */
        const std::unique_ptr<Status> statusObject;

        /** Abstraction of error monitoring */
        Error error;

        /** Error instances for watching for throttling events */
        Error throttleTemp;
        Error throttlePower;
        Error throttleMem;

        /** @brief file writer to achieve bind and unbind
         *
         *  @param[in] filename - Name of file to be written
         *  @param[in] data     - Data to be written to
         *  @return             - None
         */
        void write(const fs::path& fileName, const std::string& data)
        {
            // If there is an error, move the exception all the way up
            std::ofstream file(fileName, std::ios::out);
            file << data;
            file.close();
            return;
        }

        /** @brief callback for the proc temp throttle event
         *
         * @param[in] error - True if an error is reported, false otherwise
         */
        void throttleTempCallback(bool error);

        /** @brief callback for the proc power throttle event
         *
         * @param[in] error - True if an error is reported, false otherwise
         */
        void throttlePowerCallback(bool error);

        /** @brief callback for the proc temp throttle event
         *
         * @param[in] error - True if an error is reported, false otherwise
         */
        void throttleMemCallback(bool error);
};

} // namespace occ
} // namespace open_power
