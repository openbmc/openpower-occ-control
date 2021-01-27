#pragma once

#include "occ_errors.hpp"
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>
#include <phosphor-logging/log.hpp>
#include "occ_command.hpp"

#include <chrono>

namespace open_power
{
namespace occ
{

class Manager;

/** @class Poller
 *  @brief Monitors the state of the OCCs
 */
class Poller : public Error
{
  public:
    Poller() = delete;
    Poller(const Poller&) = delete;
    Poller& operator=(const Poller&) = delete;
    Poller(Poller&&) = default;
    Poller& operator=(Poller&&) = default;


    /** @brief Constructs the Poller object
     *
     *  @param[in] event    - Reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate errors
     *  @param[in] mgr      - OCC manager instance
     *  @param[in] callBack - Optional function callback on error condition
     */
    Poller(EventPtr& event, int instance, sdbusplus::bus::bus& bus, const fs::path& file, const Manager& mgr,
             std::function<void(bool)> callBack = nullptr) :
        Error(event, file, callBack),
        manager(mgr),
        sdp_event(sdeventplus::Event::get_default()),
        occ_instance(instance),
        interval(10),
        occ_pt(nullptr),
        _pollTimer(std::make_unique<sdeventplus::utility::
                   Timer<sdeventplus::ClockId::Monotonic>>(sdp_event,
                                                           std::bind(&Poller::pollerTimerExpired, this)
                                                           //std::chrono::seconds{10}
                                                          )
                  )
    {
        using namespace phosphor::logging;
        std::string m = "Poller::Poller(OCC" + std::to_string(instance) + ", path=" + file.c_str() + ")";
        log<level::INFO>(m.c_str());

        auto occ = std::string(OCC_NAME) + std::to_string(instance);
        auto path = fs::path(OCC_CONTROL_ROOT) / occ;
        occ_pt = new OccCommand(bus, path.c_str());
    }

    ~Poller()
    {
        if (occ_pt != nullptr) delete occ_pt;
    }

    void startPoller();

    void pollerTimerExpired();

  private:

    /** Store the manager instance to enable getting number of OCCs */
    const Manager& manager;

    sdeventplus::Event sdp_event;

    int occ_instance; // OCC instance

    uint8_t interval; // in seconds

    OccCommand* occ_pt;

    /**
     * The timer object
     */
    //sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _timer;

    /**
     * @brief The timer that uses _errorDelay.  When it expires an error
     *        will be created for a faulted fan sensor (rotor).
     *
     * If _errorDelay is std::nullopt, then this won't be created.
     */
    //sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> _pollTimer;
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
