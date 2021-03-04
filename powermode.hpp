#pragma once

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
    PowerMode(sdbusplus::bus::bus& bus, Status& occStatus,
              const std::string& occMasterName = OCC_MASTER_NAME) :
        bus(bus),
        occMasterName(occMasterName), occStatus(occStatus),
        pmodeMatch(
            bus,
            sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(
                    "/xyz/openbmc_project/control/host0/power_mode") +
                sdbusRule::argN(0, "xyz.openbmc_project.Control.Power.Mode") +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
            std::bind(std::mem_fn(&PowerMode::modeChanged), this,
                      std::placeholders::_1)){};

  private:
    /** @brief Callback for pmode setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pmode change signal
     *
     */
    void modeChanged(sdbusplus::message::message& msg);

    /** @brief Look up DBUS service for input path/interface
     *
     * @param[in]  path       - DBUS path
     * @param[in]  path       - DBUS interface
     *
     * @return Distinct service name for input path/interface
     */
    std::string getService(std::string path, std::string interface);

    /** @brief Write the input power mode to the occ hwmon entry
     *
     * @param[in]  pmodeValue - Power mode value to write to OCC
     */
    void writeMode(uint8_t pmodeValue);

    /**
     * @brief Returns the filename to use for the user power mode
     *
     * The file is of the form "powerX_mode_user", where X is any
     * number.
     *
     * @param[in] path - The directory to look for the file in
     *
     * @return std::string - The filename, or empty string if not found.
     */
    std::string
        getPmodeFilename(const std::experimental::filesystem::path& path);

    /** @brief Reference to sdbus **/
    sdbusplus::bus::bus& bus;

    /** @brief The master occ name */
    std::string occMasterName;

    /* @brief OCC Status object */
    Status& occStatus;

    /** @brief Used to subscribe to dbus pmode property changes **/
    sdbusplus::bus::match_t pmodeMatch;
};

} // namespace powermode

} // namespace occ

} // namespace open_power
