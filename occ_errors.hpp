#pragma once

#include "config.h"

#include "occ_events.hpp"

#include <unistd.h>

#include <experimental/filesystem>
#include <functional>
namespace open_power
{
namespace occ
{

namespace fs = std::experimental::filesystem;

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
          std::function<void(bool)> callBack = nullptr) :
        event(event),
        file(file), callBack(callBack)
    {
        // Nothing to do here.
    }

    ~Error()
    {
        if (fd >= 0)
        {
            close(fd);
        }
    }

    /** @brief Starts to monitor for error conditions */
    void addWatch(bool noPoll = false);

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
    std::function<void(bool)> callBack;

    /** @brief Reads file data
     *
     *  @return data read. Since its a /sysfs entry,
     *          it would be a string
     */
    std::string readFile(int) const;
};

} // namespace occ
} // namespace open_power
