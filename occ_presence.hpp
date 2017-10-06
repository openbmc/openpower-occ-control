#pragma once

#include "occ_errors.hpp"
namespace open_power
{
namespace occ
{

class Manager;

/** @class Presence
 *  @brief Monitors the number of OCCs present
 */
class Presence : public Error
{
    public:
        Presence() = delete;
        Presence(const Presence&) = delete;
        Presence& operator=(const Presence&) = delete;
        Presence(Presence&&) = default;
        Presence& operator=(Presence&&) = default;

        /** @brief Constructs the Presence object
         *
         *  @param[in] event    - Reference to sd_event unique_ptr
         *  @param[in] file     - File used by driver to communicate errors
         *  @param[in] mgr      - OCC manager instance
         *  @param[in] callBack - Optional function callback on error condition
         */
        Presence(EventPtr& event,
              const fs::path& file,
              const Manager& mgr,
              std::function<void()> callBack = nullptr) :
            Error(event, file, callBack),
            manager(mgr)
        {
            // Nothing to do here.
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
