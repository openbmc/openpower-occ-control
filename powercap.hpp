#pragma once

#include "config.h"

#include "utils.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <filesystem>
#include <regex>

namespace open_power
{
namespace occ
{
class Status;

namespace powercap
{

namespace sdbusRule = sdbusplus::bus::match::rules;
namespace fs = std::filesystem;

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
     * @param[in] occStatus - The occ status object
     */
    explicit PowerCap(Status& occStatus) :
        occStatus(occStatus),
        pcapMatch(
            utils::getBus(),
            sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(
                    "/xyz/openbmc_project/control/host0/power_cap") +
                sdbusRule::argN(0, "xyz.openbmc_project.Control.Power.Cap") +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
            std::bind(std::mem_fn(&PowerCap::pcapChanged), this,
                      std::placeholders::_1)){};

    /** @brief Return the appropriate value to write to the OCC
     *
     * @param[in]  pcap        - Current user power cap setting
     * @param[in]  pcapEnabled - Current power cap enable setting
     *
     * @return The value to write to the occ user pcap
     */
    uint32_t getOccInput(uint32_t pcap, bool pcapEnabled);

    /** @brief Read the power cap bounds from sysfs and update DBus */
    void updatePcapBounds();

  private:
    /** @brief Callback for pcap setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pcap change signal
     *
     */
    void pcapChanged(sdbusplus::message::message& msg);

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

    /** @brief Write the input power cap to the occ hwmon entry
     *
     * @param[in]  pcapValue - Power cap value to write to OCC
     */
    void writeOcc(uint32_t pcapValue);

    /** @brief Read the user power cap from sysfs
     *
     * @return User power cap value in Watts or 0 if disabled
     */
    uint32_t readUserCapHwmon();

    /**
     * @brief Returns the filename to use for the user power cap
     *
     * The file is of the form "powerX_cap_user", where X is any
     * number.
     *
     * @param[in] expr - Regular expression of file to find
     *
     * @return full path/filename, or empty path if not found.
     */
    fs::path getPcapFilename(const std::regex& expr);

    /* @brief OCC Status object */
    Status& occStatus;

    /** @brief Used to subscribe to dbus pcap property changes **/
    sdbusplus::bus::match_t pcapMatch;

    /** @brief Path to the sysfs files holding the cap properties **/
    fs::path pcapBasePathname;

    /** @brief Update the power cap bounds on DBus
     *
     * @param[in]  hardMin - hard minimum power cap in Watts
     * @param[in]  pcapMax - maximum power cap in Watts
     *
     * @return true if all parms were written successfully
     */
    bool updateDbusPcap(uint32_t hardMin, uint32_t pcapMax);
};

} // namespace powercap

} // namespace occ

} // namespace open_power
