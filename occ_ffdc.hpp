#pragma once

#include "config.h"

#include "occ_errors.hpp"

namespace open_power
{
namespace occ
{

/** @class FFDC
 *  @brief Monitors for SBE FFDC availability
 */
class FFDC : public Error
{
  public:
    FFDC() = delete;
    FFDC(const FFDC&) = delete;
    FFDC& operator=(const FFDC&) = delete;
    FFDC(FFDC&&) = default;
    FFDC& operator=(FFDC&&) = default;

    /** @brief Constructs the FFDC object
     *
     *  @param[in] event    - reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate FFDC data
     *  @param[in] instance - OCC instance number
     */
    FFDC(EventPtr& event, const fs::path& file, unsigned int instance) :
        Error(event, file, nullptr), instance(instance)
    {
        // Nothing to do here.
    }

    ~FFDC()
    {
        for (auto&& it : temporaryFiles)
        {
            close(it.second);
            fs::remove(it.first);
        }
    }

    /** @brief Helper function to create a PEL with the OpenPower DBus
     *         interface
     *
     * @param[in] path - the DBus error path
     * @param[in] src6 - the SBE error SRC6 word
     * @param[in] msg - the error message
     * @param[in] fd - the file descriptor for any FFDC
     */
    static uint32_t createPEL(const char* path, uint32_t src6, const char* msg,
                              int fd = -1);

  private:
    /** @brief OCC instance number. Ex, 0,1, etc */
    unsigned int instance;

    /** @brief Stores the temporary files and file descriptors
     *         in usage. They will be cleaned up when the class
     *         is destroyed (when the application exits).
     */
    std::vector<std::pair<fs::path, int>> temporaryFiles;

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;
};

} // namespace occ
} // namespace open_power
