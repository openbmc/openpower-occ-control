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
     */
    FFDC(EventPtr& event, const fs::path& file) : Error(event, file, nullptr)
    {
        // Nothing to do here.
    }

  private:
    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;
};

} // namespace occ
} // namespace open_power
