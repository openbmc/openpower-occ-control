#pragma once

#include "config.h"

#include "utils.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <xyz/openbmc_project/Control/Power/CapLimits/server.hpp>

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

constexpr auto PCAPLIMITS_PATH =
    "/xyz/openbmc_project/control/host0/power_cap_limits";

namespace Base = sdbusplus::xyz::openbmc_project::Control::Power::server;
using CapLimitsInterface = sdbusplus::server::object_t<Base::CapLimits>;

constexpr auto PCAPDATA_FILE_VERSION = 1;
struct PowerCapData
{
    uint32_t version = PCAPDATA_FILE_VERSION;
    bool initialized = false;
    uint32_t softMin = 0x0000;
    uint32_t hardMin = 0x0000;
    uint32_t max = UINT_MAX;
};

/** @class OccPersistCapData
 *  @brief Provides persistent container to store data for OCC
 *
 * Data is stored in filesystem
 */
class OccPersistCapData
{
  public:
    ~OccPersistCapData() = default;
    OccPersistCapData(const OccPersistCapData&) = default;
    OccPersistCapData& operator=(const OccPersistCapData&) = default;
    OccPersistCapData(OccPersistCapData&&) = default;
    OccPersistCapData& operator=(OccPersistCapData&&) = default;

    /** @brief Loads any saved power cap data */
    OccPersistCapData()
    {
        load();
    }

    /** @brief Save Power Mode data to persistent file
     *
     *  @param[in] softMin - soft minimum power cap in Watts
     *  @param[in] hardMin - hard minimum power cap in Watts
     *  @param[in] max     - maximum power cap in Watts
     */
    void updateCapLimits(const uint32_t softMin, const uint32_t hardMin,
                         const uint32_t max)
    {
        capData.softMin = softMin;
        capData.hardMin = hardMin;
        capData.max = max;
        capData.initialized = true;
        save();
    }

    /** @brief Return the power cap limits
     *
     *  @param[out] softMin - soft minimum power cap in Watts
     *  @param[out] hardMin - hard minimum power cap in Watts
     *  @param[out] max     - maximum power cap in Watts
     */
    void getCapLimits(uint32_t& softMin, uint32_t& hardMin, uint32_t& max) const
    {
        // If not initialized yet, still return PowerCapData defaults
        softMin = capData.softMin;
        hardMin = capData.hardMin;
        max = capData.max;
    }

    /** @brief Return true if the power cap limits are available */
    bool limitsAvailable()
    {
        return (capData.initialized);
    }

    /** @brief Saves the Power Mode data in the filesystem. */
    void save();

    /** @brief Trace the Power Mode and IPS parameters. */
    void print();

  private:
    /** @brief Power Mode data filename to store persistent data */
    static constexpr auto powerCapFilename = "powerCapLimitData";

    /** @brief Power Mode data object to be persisted */
    PowerCapData capData;

    /** @brief Loads the persisted power cap data from the filesystem. */
    void load();
};

/** @class PowerCap
 *  @brief Monitors for changes to the power cap and notifies occ
 *
 *  The customer power cap is provided to the OCC by host TMGT when the occ
 *  first goes active or is reset.  This code is responsible for sending
 *  the power cap to the OCC if the cap is changed while the occ is active.
 */

class PowerCap : public CapLimitsInterface
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
        CapLimitsInterface(utils::getBus(), PCAPLIMITS_PATH,
                           CapLimitsInterface::action::defer_emit),
        occStatus(occStatus),
        pcapMatch(
            utils::getBus(),
            sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(
                    "/xyz/openbmc_project/control/host0/power_cap") +
                sdbusRule::argN(0, "xyz.openbmc_project.Control.Power.Cap") +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
            std::bind(std::mem_fn(&PowerCap::pcapChanged), this,
                      std::placeholders::_1))
    {
        //  Read the current limits from persistent data
        uint32_t capSoftMin, capHardMin, capMax;
        persistedData.getCapLimits(capSoftMin, capHardMin, capMax);
        // Update limits on dbus
        updateDbusPcapLimits(capSoftMin, capHardMin, capMax);
        // CapLimit interface is now ready
        this->emit_object_added();
    };

    /** @brief Return the appropriate value to write to the OCC (output/DC
     * power)
     *
     * @param[in]  pcap        - Current user power cap setting (input/AC power)
     * @param[in]  pcapEnabled - Current power cap enable setting
     *
     * @return The value to write to the occ user pcap
     */
    uint32_t getOccInput(uint32_t pcap, bool pcapEnabled);

    /** @brief Read the power cap bounds from sysfs and update DBus */
    void updatePcapBounds();

  private:
    /** @brief Persisted power cap limits */
    OccPersistCapData persistedData;

    /** @brief Callback for pcap setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pcap change signal
     *
     */
    void pcapChanged(sdbusplus::message_t& msg);

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

    /** @brief Write the output/DC power cap to the occ hwmon entry
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
     * @param[in]  softMin - soft minimum power cap in Watts
     * @param[in]  hardMin - hard minimum power cap in Watts
     * @param[in]  pcapMax - maximum power cap in Watts
     */
    void updateDbusPcapLimits(uint32_t softMin, uint32_t hardMin,
                              uint32_t pcapMax);
};

} // namespace powercap

} // namespace occ

} // namespace open_power
