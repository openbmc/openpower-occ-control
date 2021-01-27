#pragma once

#include "occ_command.hpp"
#include "occ_errors.hpp"

#include <fmt/core.h>

#include <chrono>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>

namespace open_power
{
namespace occ
{

class Manager;

/** @brief Default time, in seconds, between poll commands */
constexpr unsigned int defaultPollingInterval = 10;

/** @class Poller
 *  @brief Monitors the state of the OCC by periodic polling
 */
class Poller : public Error
{
  public:
    Poller() = delete;
    ~Poller() = default;
    Poller(const Poller&) = delete;
    Poller& operator=(const Poller&) = delete;
    Poller(Poller&&) = default;
    Poller& operator=(Poller&&) = default;

    /**
     * @brief Constructs the Poller object
     *
     *  @param[in] event    - Reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate errors
     *  @param[in] mgr      - OCC manager instance
     *  @param[in] callBack - Optional function callback on error condition
     */
    Poller(EventPtr& event, int instance, sdbusplus::bus::bus& bus,
           const fs::path& file, const Manager& mgr,
           std::function<void(bool)> callBack = nullptr) :
        Error(event, file, callBack),
        manager(mgr), sdp_event(sdeventplus::Event::get_default()),
        occ_instance(instance), interval(defaultPollingInterval),
        _pollTimer(
            std::make_unique<
                sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>(
                sdp_event, std::bind(&Poller::pollerTimerExpired, this)))
    {
        // Nothing to do here.
    }

    /** @brief Start timer that will trigger a poll when it expires. This will
     *         intially get called when an OCC is reported as going active. */
    void startPoller();

    /**
     * @brief Called when timer expires. Sends a POLL command to the OCC and
     *        process the response. The timer will then be restarted.
     * */
    void pollerTimerExpired();

  private:
    /** @brief Store the manager instance to enable getting number of OCCs */
    const Manager& manager;

    /** @brief Object to monitor OCC device errors */
    sdeventplus::Event sdp_event;

    /** @brief OCC instance number */
    int occ_instance;

    /** @brief Number of seconds between poll commands */
    uint8_t interval;

    /**
     * @brief The timer to be used once the OCC goes active.  When it expires,
     *        a POLL command will be sent to the OCC and then timer restarted.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        _pollTimer;

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;
};

} // namespace occ
} // namespace open_power
