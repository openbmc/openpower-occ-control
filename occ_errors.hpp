#pragma once

#include "config.h"

#include "occ_events.hpp"

#include <unistd.h>

#include <filesystem>
#include <functional>
namespace open_power
{
namespace occ
{

namespace fs = std::filesystem;

constexpr auto PRESENCE_ERROR_PATH =
    "org.open_power.OCC.Firmware.Error.PresenceMismatch";
constexpr auto SAFE_ERROR_PATH = "org.open_power.OCC.Device.Error.SafeState";
constexpr auto MISSING_OCC_SENSORS_PATH =
    "org.open_power.OCC.Firmware.Error.MissingOCCSensors";
constexpr auto OCC_COMM_ERROR_PATH =
    "org.open_power.OCC.Device.Error.OpenFailure";

/** @class Error
 *  @brief Monitors for OCC device error condition
 */
class Error
{
  public:
    Error() = delete;
    Error(const Error&) = delete;
    Error& operator=(const Error&) = delete;
    Error(Error&&) = default;
    Error& operator=(Error&&) = default;

    /** @brief Constructs the Error object
     *
     *  @param[in] event    - reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate errors
     *  @param[in] callBack - Optional function callback on error condition
     */
    Error(EventPtr& event, const fs::path& file,
          std::function<void(int)> callBack = nullptr) :
        event(event), file(file), callBack(callBack)
    {
        // Nothing to do here.
    }

    virtual ~Error()
    {
        if (fd >= 0)
        {
            close(fd);
        }
    }

    /** @class Descriptor
     *  @brief Contains data relevant to an error that occurred.
     */
    class Descriptor
    {
      public:
        Descriptor(const Descriptor&) = default;
        Descriptor& operator=(const Descriptor&) = default;
        Descriptor(Descriptor&&) = default;
        Descriptor& operator=(Descriptor&&) = default;

        Descriptor() : log(false), err(0), callout(nullptr), path(nullptr) {}

        /** @brief Constructs the Descriptor object
         *
         *  @param[in] path - the DBus error path
         *  @param[in] err - Optional error return code
         *  @param[in] callout - Optional PEL callout path
         *  @param[in] isInventory - true if the callout path is an
         * inventory path, false if it is a device path
         */
        Descriptor(const char* path, int err = 0, const char* callout = nullptr,
                   const bool isInventory = false) :
            log(true), err(err), callout(callout), path(path),
            isInventoryCallout(isInventory)
        {}

        bool log;
        int err;
        const char* callout;
        const char* path;
        bool isInventoryCallout;
    };

    /** @brief Starts to monitor for error conditions
     *
     *  @param[in] poll - Indicates whether or not the error file should
     *                    actually be polled for changes. Disabling polling is
     *                    necessary for error files that don't support the poll
     *                    file operation.
     */
    void addWatch(bool poll = true);

    /** @brief Removes error watch */
    void removeWatch();

    inline void setFile(const fs::path& f)
    {
        file = f;
    }

  private:
    /** @brief sd_event wrapped in unique_ptr */
    EventPtr& event;

    /** @brief event source wrapped in unique_ptr */
    EventSourcePtr eventSource;

    /** @brief Current state of error watching */
    bool watching = false;

    /** @brief attaches FD to events and sets up callback handler */
    void registerCallBack();

    /** @brief Opens the file and populates fd */
    void openFile();

    /** @brief Callback handler when the FD has some activity on it
     *
     *  @param[in] es       - Populated event source
     *  @param[in] fd       - Associated File descriptor
     *  @param[in] revents  - Type of event
     *  @param[in] userData - User data that was passed during registration
     *
     *  @return             - 0 or positive number on success and negative
     *                        errno otherwise
     */
    static int processEvents(sd_event_source* es, int fd, uint32_t revents,
                             void* userData);

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    virtual void analyzeEvent();

  protected:
    /** @brief File descriptor to watch for errors */
    int fd = -1;

    /** Error file */
    fs::path file;

    /** @brief Optional function to call on error scenario */
    std::function<void(int)> callBack;

    /** @brief Reads file data
     *
     *  @return data read. Since its a /sysfs entry,
     *          it would be a string
     */
    std::string readFile(int) const;
};

} // namespace occ
} // namespace open_power
