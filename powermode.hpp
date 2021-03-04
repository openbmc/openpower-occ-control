#pragma once

#ifdef POWER10
#include "config.h"

#include "occ_status.hpp"

#include <experimental/filesystem>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

namespace open_power
{
namespace occ
{
namespace powermode
{

namespace sdbusRule = sdbusplus::bus::match::rules;

constexpr auto PMODE_PATH = "/xyz/openbmc_project/control/host0/power_mode";
constexpr auto PMODE_INTERFACE = "xyz.openbmc_project.Control.Power.Mode";
constexpr auto POWER_MODE_PROP = "PowerMode";

/** @brief Convert power mode string to OCC SysPwrMode value
 *
 * @param[in] i_modeString - power mode string
 *
 * @return  SysPwrMode or SysPwrMode::NO_CHANGE if not found
 */
SysPwrMode convertStringToMode(const std::string& i_modeString);

/** @class PowerMode
 *  @brief Monitors for changes to the power mode and notifies occ
 *
 *  The customer power mode is provided to the OCC by host TMGT when the occ
 *  first goes active or is reset.  This code is responsible for sending
 *  the power mode to the OCC if the mode is changed while the occ is active.
 */

class PowerMode
{
  public:
    /** @brief PowerMode object to inform occ of changes to mode
     *
     * This object will monitor for changes to the power mode setting.
     * If a change is detected, and the occ is active, then this object will
     * notify the OCC of the change.
     *
     * @param[in] bus       - The Dbus bus object
     * @param[in] occStatus - The occ status object
     */
    PowerMode(sdbusplus::bus::bus& bus, Status& occStatus) :
        bus(bus), occStatus(occStatus),
        pmodeMatch(bus,
                   sdbusplus::bus::match::rules::propertiesChanged(
                       PMODE_PATH, PMODE_INTERFACE),
                   [this](auto& msg) { this->modeChanged(msg); }){};

  private:
    /** @brief Callback for pmode setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pmode change signal
     *
     */
    void modeChanged(sdbusplus::message::message& msg);

    /** @brief Reference to sdbus **/
    sdbusplus::bus::bus& bus;

    /* @brief OCC Status object */
    Status& occStatus;

    /** @brief Used to subscribe to dbus pmode property changes **/
    sdbusplus::bus::match_t pmodeMatch;
};

} // namespace powermode

} // namespace occ

} // namespace open_power
#endif
