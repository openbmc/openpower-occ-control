#pragma once

#include <string>
#include <experimental/filesystem>

namespace open_power
{
namespace occ
{

namespace fs = std::experimental::filesystem;

/** @class Bus
 *  @brief Manages the bus on which the OCC device exists
 */
class Bus
{
    public:
        Bus() = delete;
        ~Bus() = default;
        Bus(const Bus&) = delete;
        Bus& operator=(const Bus&) = delete;
        Bus(Bus&&) = default;
        Bus& operator=(Bus&&) = default;

        /** @brief Constructs the Bus object
         *
         *  @param[in] instance - OCC device index
         */
        explicit Bus(int instance) :
            config("0" + std::to_string(instance) + ":0" +
                std::to_string(instance) + ":00:06")
        {
            // Nothing to do here
        }

        void reset() const;

    private:
        /** @brief Config value to be used to do bind and unbind */
        const std::string config;

        /**  @brief To bind the device to the OCC's bus driver, do:
         *
         *    Write 0x:0x:00:06 to: /sys/bus/fsi/drivers/sbefifo/bind
         */
        static fs::path bindPath;

        /**  @brief To un-bind the dvice from the OCC's bus driver, do:
         *    Write 0x:0x:00:06 to: /sys/bus/fsi/drivers/sbefifo/unbind
         */
        static fs::path unBindPath;
};

} // namespace occ
} // namespace open_power
