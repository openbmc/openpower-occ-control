#pragma once

#include "config.h"

#ifdef POWER10
#include "occ_command.hpp"

#include <cereal/archives/json.hpp>
//#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/vector.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

#include <filesystem>

namespace open_power
{
namespace occ
{

class Manager;

namespace powermode
{

constexpr auto PMODE_PATH = "/xyz/openbmc_project/control/host0/power_mode";
constexpr auto PMODE_INTERFACE = "xyz.openbmc_project.Control.Power.Mode";
constexpr auto POWER_MODE_PROP = "PowerMode";

constexpr auto PIPS_PATH = "/xyz/openbmc_project/control/host0/power_ips";
constexpr auto PIPS_INTERFACE =
    "xyz.openbmc_project.Control.Power.IdlePowerSaver";
constexpr auto IPS_ENABLED_PROP = "Enabled";
constexpr auto IPS_ENTER_UTIL = "EnterUtilizationPercent";
constexpr auto IPS_ENTER_TIME = "EnterDwellTime";
constexpr auto IPS_EXIT_UTIL = "ExitUtilizationPercent";
constexpr auto IPS_EXIT_TIME = "ExitDwellTime";

/** @brief Query the current Hypervisor target
 * @return true if the current Hypervisor target is PowerVM
 */
bool isPowerVM();

/** @brief Convert power mode string to OCC SysPwrMode value
 *
 * @param[in] i_modeString - power mode string
 *
 * @return  SysPwrMode or SysPwrMode::NO_CHANGE if not found
 */
SysPwrMode convertStringToMode(const std::string& i_modeString);

struct OemModeData
{
    SysPwrMode oemMode = SysPwrMode::NO_CHANGE;
    uint16_t oemModeFreq = 0x0000;

    /** @brief Function specifying data to archive for cereal.
     */
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(oemMode, oemModeFreq);
    }
};

/** @class OccPersistData
 *  @brief Provides persistent container to store data for OCC
 *
 * Data is stored via cereal
 */
class OccPersistData
{
  public:
    ~OccPersistData() = default;
    OccPersistData(const OccPersistData&) = default;
    OccPersistData& operator=(const OccPersistData&) = default;
    OccPersistData(OccPersistData&&) = default;
    OccPersistData& operator=(OccPersistData&&) = default;

    /** @brief Loads any saved OEM mode data */
    OccPersistData()
    {
        load();
    }

    /** @brief Save Power Mode data to persistent file
     *
     *  @param[in] newMode - desired OEM Power Mode
     *  @param[in] modeData - data required by some OEM Power Modes
     */
    void writeModeFile(const SysPwrMode newMode, const uint16_t modeData)
    {
        oemData.oemMode = newMode;
        oemData.oemModeFreq = modeData;
        oemSet = true;
        save();
    }

    /** @brief Return the OEM Power Mode and frequency if enabled
     *
     *  @param[out] newMode - OEM mode (if set, else data not changed)
     *  @param[out] oemFreq - Frequency data for OEM mode
     *
     *  @returns true if OEM mode was set
     */
    bool getOemMode(SysPwrMode& mode, uint16_t& freq) const
    {
        if (!oemSet)
        {
            return false;
        }

        mode = oemData.oemMode;
        freq = oemData.oemModeFreq;
        return true;
    }

    /** @brief Saves the Power Mode data in the filesystem using cereal. */
    void save();

    /** @brief Removes the OEM mode data. */
    void purge();

    inline void print();

  private:
    static constexpr auto oemModeFilename = "oemModeData";

    /** @brief true if an OEM Power Mode was set */
    bool oemSet = false;

    /** @brief OEM Power Mode data */
    OemModeData oemData;

    /** @brief Loads the OEM mode data in the filesystem using cereal. */
    void load();
};

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
     * @param[in] managerRef -
     * @param[in] path -
     */
    explicit PowerMode(const Manager& managerRef) :
        manager(managerRef), occInstance(0),
        pmodeMatch(utils::getBus(),
                   sdbusplus::bus::match::rules::propertiesChanged(
                       PMODE_PATH, PMODE_INTERFACE),
                   [this](auto& msg) { this->modeChanged(msg); }),
        ipsMatch(utils::getBus(),
                 sdbusplus::bus::match::rules::propertiesChanged(
                     PIPS_PATH, PIPS_INTERFACE),
                 [this](auto& msg) { this->ipsChanged(msg); }),
        masterOccSet(false), masterActive(false){};

    bool setMode(const SysPwrMode newMode, const uint16_t modedata);

    /** @brief Send mode change command to the master OCC
     *  @return SUCCESS on success
     */
    CmdStatus sendModeChange();

    /** @brief Send Idle Power Saver config data to the master OCC
     *  @return SUCCESS on success
     */
    CmdStatus sendIpsData();

    /** @brief Set the master OCC path
     *
     * @param[in]  occPath - hwmon path for master OCC
     */
    void setMasterOcc(const std::string& occPath);

    /** @brief Notify object of master OCC state.  If not acitve, no
     * commands will be sent to the master OCC
     *
     * @param[in]  isActive - true when master OCC is active
     */
    void setMasterActive(const bool isActive = true)
    {
        masterActive = isActive;
    };

  private:
    /** @brief OCC manager object */
    const Manager& manager;

    /** @brief Pass-through occ path on the bus */
    std::string path;

    /** @brief OCC instance number */
    int occInstance;

    /** @brief Object to send commands to the OCC */
    std::unique_ptr<open_power::occ::OccCommand> occCmd;

    /** @brief Used to subscribe to dbus pmode property changes **/
    sdbusplus::bus::match_t pmodeMatch;

    /** @brief Used to subscribe to dbus IPS property changes **/
    sdbusplus::bus::match_t ipsMatch;

    OccPersistData persistedData;

    /** @brief True when the master OCC has been established */
    bool masterOccSet;

    /** @brief True when the master OCC is active */
    bool masterActive;

    /** @brief Callback for pmode setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg       - Data associated with pmode change signal
     *
     */
    void modeChanged(sdbusplus::message::message& msg);

    /** @brief Get the current power mode property from DBus
     * @return Power mode
     */
    SysPwrMode getDbusMode();

    /** @brief Update the power mode property on DBus
     *
     * @param[in]  newMode - desired power mode
     *
     * @return true on success
     */
    bool updateDbusMode(const SysPwrMode newMode);

    /** @brief Callback for IPS setting changes
     *
     * Process change and inform OCC
     *
     * @param[in]  msg - Data associated with IPS change signal
     *
     */
    void ipsChanged(sdbusplus::message::message& msg);

    /** @brief Get the Idle Power Saver properties from DBus
     * @return true if IPS is enabled
     */
    bool getIPSParms(uint8_t& enterUtil, uint16_t& enterTime, uint8_t& exitUtil,
                     uint16_t& exitTime);
};

} // namespace powermode

} // namespace occ

} // namespace open_power
#endif
