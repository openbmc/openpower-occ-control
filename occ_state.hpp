#pragma once

#include "occ_command.hpp"
#include "occ_errors.hpp"

#include <fmt/core.h>

#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{

class Manager;

using namespace phosphor::logging;

/** @class StateChange
 *  @brief Monitors the state of the OCCs
 */
class StateChange : public Error
{
  public:
    StateChange() = delete;
    StateChange(const StateChange&) = delete;
    StateChange& operator=(const StateChange&) = delete;
    StateChange(StateChange&&) = default;
    StateChange& operator=(StateChange&&) = default;

    /** @brief Constructs the StateChange object
     *
     *  @param[in] event    - Reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate errors
     *  @param[in] mgr      - OCC manager instance
     *  @param[in] callBack - Optional function callback on error condition
     */
    StateChange(EventPtr& event, const fs::path& file, const Manager& mgr,
                std::function<void(bool)> callBack = nullptr) :
        Error(event, file, callBack),
        manager(mgr)
    {
        // Nothing to do here.
        log<level::INFO>(
            fmt::format("StateChange(file={})", file.c_str()).c_str());
    }

  private:
    /** Store the manager instance to enable getting number of OCCs */
    const Manager& manager;

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;
};

} // namespace occ
} // namespace open_power
