#pragma once

#include "config.h"

#include "occ_errors.hpp"
#include "occ_events.hpp"
#include "occ_ffdc.hpp"
#include "occ_presence.hpp"
#include "powermode.hpp"

#include <org/open_power/OCC/Device/error.hpp>

#include <filesystem>
#include <fstream>
#include <regex>

namespace open_power
{
namespace occ
{

class Manager;
class Status;
namespace fs = std::filesystem;
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
     *  @param[in] path     - Path to the OCC instance
     *  @param[in] manager  - OCC manager instance
     *  @param[in] status   - Status instance
     *  @param[in] instance - OCC instance number
     */
    Device(EventPtr& event, const fs::path& path, Manager& manager,
           Status& status,
           unsigned int instance = 0) :
        devPath(path), instance(instance), statusObject(status),
        managerObject(manager),
        error(event, path / "occ_error",
              std::bind(std::mem_fn(&Device::errorCallback), this,
                        std::placeholders::_1)),
        timeout(event,
                path /
                    fs::path("../../sbefifo" + std::to_string(instance + 1)) /
                    "timeout", nullptr),
        ffdc(event, path / "ffdc", instance),
        presence(event, path / "occs_present", manager,
                 std::bind(std::mem_fn(&Device::errorCallback), this,
                           std::placeholders::_1)),
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

    /** @brief Sets the device active or inactive
     *
     * @param[in] active - Indicates whether or not to set the device active
     */
    void setActive(bool active);

    /** @brief Starts to monitor for errors
     *
     *  @param[in] poll - Indicates whether or not the error file should
     *                    actually be polled for changes. Disabling polling is
     *                    necessary for error files that don't support the poll
     *                    file operation.
     */
    inline void addErrorWatch(bool poll = true)
    {
        try
        {
            throttleProcTemp.addWatch(poll);
        }
        catch (const OpenFailure& e)
        {
            // try the old kernel version
            throttleProcTemp.setFile(devPath / "occ_dvfs_ot");
            throttleProcTemp.addWatch(poll);
        }

        throttleProcPower.addWatch(poll);
        throttleMemTemp.addWatch(poll);

        try
        {
            ffdc.addWatch(poll);
        }
        catch (const OpenFailure& e)
        {
            // nothing to do if there is no FFDC file
        }

        try
        {
            timeout.addWatch(poll);
        }
        catch (const std::exception& e)
        {
            // nothing to do if there is no SBE timeout file
        }

        error.addWatch(poll);
    }

    /** @brief stops monitoring for errors */
    inline void removeErrorWatch()
    {
        // we can always safely remove watch even if we don't add it
        presence.removeWatch();
        ffdc.removeWatch();
        error.removeWatch();
        timeout.removeWatch();
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

    /** @brief helper function to get the last part of the path
     *
     * @param[in] path - Path to parse
     * @return         - Last directory name in the path
     */
    static std::string getPathBack(const fs::path& path);

    /** @brief Returns true if the device is active */
    bool active() const;

    /** @brief Returns true if device represents the master OCC */
    bool master() const;

  private:
    /** @brief This directory contains the error files */
    const fs::path devPath;

    /** @brief OCC instance ID */
    const unsigned int instance;

    /**  Store the associated Status instance */
    Status& statusObject;

    /** Store the parent Manager instance */
    Manager& managerObject;

    /** Abstraction of error monitoring */
    Error error;

    /** Abstraction of SBE timeout monitoring */
    Error timeout;

    /** SBE FFDC monitoring */
    FFDC ffdc;

    /** Abstraction of OCC presence monitoring */
    Presence presence;

    /** Error instances for watching for throttling events */
    Error throttleProcTemp;
    Error throttleProcPower;
    Error throttleMemTemp;


    /** @brief file reader to read a binary string ("1" or "0")
     *
     * @param[in] fileName - Name of file to be read
     * @return             - The value returned by reading the file
     */
    bool readBinary(const std::string& fileName) const;

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

    /** @brief callback for OCC error monitoring
     *
     * @param[in] error - Errno stored in the error file, 0 if no error
     */
    void errorCallback(int error);

    /** @brief callback for OCC presence monitoring
     *
     * @param[in] occsPresent - The number of OCCs indicated in the poll
     * response
     */
    void presenceCallback(int occsPresent);

    /** @brief callback for the proc temp throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleProcTempCallback(int error);

    /** @brief callback for the proc power throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleProcPowerCallback(int error);

    /** @brief callback for the proc temp throttle event
     *
     *  @param[in] error - True if an error is reported, false otherwise
     */
    void throttleMemTempCallback(int error);

    /** @brief Get the pathname for a file based on a regular expression
     *
     *  @param[in] basePath - The path where the files will be checked
     *  @param[in] expr - Regular expression describing the target file
     *
     *  @return path to the file or empty path if not found
     */
    fs::path getFilenameByRegex(fs::path basePath,
                                const std::regex& expr) const;
};

} // namespace occ
} // namespace open_power
