#pragma once

#include <unistd.h>
#include <functional>
#include <experimental/filesystem>
#include "occ_events.hpp"
#include "config.h"
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
        Error(EventPtr& event,
              const fs::path& file,
              std::function<void()> callBack = nullptr) :
            event(event),
            file(fs::path(DEV_PATH) / file),
            callBack(callBack)
        {
            // Nothing to do here.
        }

        ~Error()
        {
            if (fd>= 0)
            {
                close(fd);
            }
        }

        /** @brief Starts to monitor for error conditions */
        void addWatch();

        /** @brief Removes error watch */
        void removeWatch();

    private:
        /** @brief sd_event wrapped in unique_ptr */
        EventPtr& event;

        /** @brief event source wrapped in unique_ptr */
        EventSourcePtr eventSource;

        /** Error file */
        const fs::path file;

        /** @brief Optional function to call on error scenario */
        std::function<void()> callBack;

        /** @brief File descriptor to watch for errors */
        int fd = -1;

        /** @brief attaches FD to events and sets up callback handler */
        void registerCallBack();

        /** @brief Opens the file and populates fd */
        void openFile();

        /** @brief Reads file data
         *
         *  @return data read. Since its a /sysfs entry,
         *          it would be a string
         */
        std::string readFile(int) const;

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
        static int processEvents(sd_event_source* es, int fd,
                                 uint32_t revents, void* userData);

        void analyzeEvent();
};

} // namespace occ
} // namespace open_power
