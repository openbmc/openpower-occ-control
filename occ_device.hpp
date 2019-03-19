#pragma once

#include "config.h"

#include "occ_errors.hpp"
#include "occ_events.hpp"
#include "occ_presence.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <org/open_power/OCC/Device/error.hpp>

namespace open_power
{
namespace occ
{

class Manager;
class Status;
namespace fs = std::experimental::filesystem;
using namespace sdbusplus::org::open_power::OCC::Device::Error;

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
     *  @param[in] manager  - OCC manager instance
     *  @param[in] callback - Optional callback on errors
     */
    Device(EventPtr& event, const fs::path& path, const Manager& manager,
           Status& status, std::function<void(bool)> callBack = nullptr) :
        config(get_path_back(path)),
        devPath(path), statusObject(status),
        error(event, path / "occ_error", callBack),
        presence(event, path / "occs_present", manager, callBack),
        throttleProcTemp(
            event, path / "occ_dvfs_overtemp",
            std::bind(std::mem_fn(&Device::throttleProcTempCallback), this,
                      std::placeholders::_1)),
        throttleProcPower(
            event, path / "occ_dvfs_power",
            std::bind(std::mem_fn(&Device::throttleProcPowerCallback), this,
                      std::placeholders::_1)),
        throttleMemTemp(event, path / "occ_mem_throttle",
                        std::bind(std::mem_fn(&Device::throttleMemTempCallback),
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

    /** @brief Starts to monitor for errors
     *
     *  @param[in] noPoll - Indicates whether or not the error file should
     *                      actually be polled for changes. Disabling polling is
     *                      necessary for error files that don't support the
                            poll file operation.
     */
    inline void addErrorWatch(bool noPoll = false)
    {
        try
        {
            throttleProcTemp.addWatch(noPoll);
        }
        catch (const OpenFailure& e)
        {
            // try the old kernel version
            throttleProcTemp.setFile(devPath / "occ_dvfs_ot");
            throttleProcTemp.addWatch(noPoll);
        }

        throttleProcPower.addWatch(noPoll);
        throttleMemTemp.addWatch(noPoll);
        error.addWatch(noPoll);
    }

    /** @brief stops monitoring for errors */
    inline void removeErrorWatch()
    {
        // we can always safely remove watch even if we don't add it
        presence.removeWatch();
        error.removeWatch();
        throttleMemTemp.removeWatch();
        throttleProcPower.removeWatch();
        throttleProcTemp.removeWatch();
    }

    /** @brief Starts to watch how many OCCs are present on the master */
    inline void addPresenceWatchMaster()
    {
        if (master())
        {
            presence.addWatch();
        }
    }

  private:
    /** @brief Config value to be used to do bind and unbind */
    const std::string config;

    /** @brief This directory contains the error files */
    const fs::path devPath;

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
    Status& statusObject;

    /** Abstraction of error monitoring */
    Error error;

    /** Abstraction of OCC presence monitoring */
    Presence presence;

    /** Error instances for watching for throttling events */
    Error throttleProcTemp;
    Error throttleProcPower;
    Error throttleMemTemp;

    /** @brief helper function to get the last part of the path
     *
     * @param[in] path - Path to parse
     * @return         - Last directory name in the path
     */
    std::string get_path_back(const fs::path& path)
    {
        if (path.empty())
            return std::string();

        auto conf = --path.end(); // this points to the last element in the path
        if (conf->empty() && conf != path.begin())
        {
            // last element is empty so go back one more
            return *(--conf);
        }
        else
        {
            return *conf;
        }
    }

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

    /** @brief Returns if device represents the master OCC */
    bool master() const;

    /** @brief callback for the proc temp throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleProcTempCallback(bool error);

    /** @brief callback for the proc power throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleProcPowerCallback(bool error);

    /** @brief callback for the proc temp throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleMemTempCallback(bool error);
};

} // namespace occ
} // namespace open_power
