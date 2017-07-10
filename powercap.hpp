#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include "occ_status.hpp"
#include "config.h"

namespace open_power
{
namespace occ
{
namespace powercap
{

namespace sdbusRule = sdbusplus::bus::match::rules;

/** @class PowerCap
 *  @brief Monitors for changes to the power cap and notifies occ
 *
 *  The customer power cap is provided to the OCC by host TMGT when the occ
 *  first goes active or is reset.  This code is responsible for sending
 *  the power cap to the OCC if the cap is changed while the occ is active.
 */

class PowerCap
{
public:
    /** @brief PowerCap object to inform occ of changes to cap
     *
     * This object will monitor for changes to the power cap setting and
     * power cap enable properties.  If a change is detected, and the occ
     * is active, then this object will notify the OCC of the change.
     *
     * @param[in] bus       - The Dbus bus object
     * @param[in] occStatus - The occ status object
     */
    PowerCap(sdbusplus::bus::bus &bus,
             Status &occStatus) :
        bus(bus),
        occStatus(occStatus),
        pcapMatch(
                bus,
                sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(
                    "/xyz/openbmc_project/control/host0/power_cap") +
                sdbusRule::argN(0, "xyz.openbmc_project.Control.Power.Cap") +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
                std::bind(std::mem_fn(&PowerCap::pcapChanged),
                          this, std::placeholders::_1))
    {};

    /** @brief Return the appropriate value to write to the OCC
     *
     * @param[in]  pcap        - Current user power cap setting
     * @param[in]  pcapEnabled - Current power cap enable setting
     *
     * @return The value to write to the occ user pcap
     */
    uint32_t getOccInput(uint32_t pcap, bool pcapEnabled);

private:

    /** @brief Callback for pcap setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pcap change signal
     *
     */
    void pcapChanged(sdbusplus::message::message& msg);

    /** @brief Look up DBUS service for input path/interface
     *
     * @param[in]  path       - DBUS path
     * @param[in]  path       - DBUS interface
     *
     * @return Distinct service name for input path/interface
     */
    std::string getService(std::string path,
                           std::string interface);

    /** @brief Get the power cap property
     *
     * @return Power cap, 0 on failure to indicate no pcap
     */
    uint32_t getPcap();

    /** @brief Get the power cap enable property
     *
     * @return Whether power cap enabled, will return false on error
     */
    bool getPcapEnabled();

    /** @brief Reference to sdbus **/
    sdbusplus::bus::bus& bus;

    /* @brief OCC Status object */
    Status &occStatus;

    /** @brief Used to subscribe to dbus pcap propety changes **/
    sdbusplus::bus::match_t pcapMatch;

 };

} // namespace open_power

} // namespace occ

}// namespace powercap
