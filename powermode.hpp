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
#include <xyz/openbmc_project/Control/Power/IdlePowerSaver/server.hpp>
#include <xyz/openbmc_project/Control/Power/Mode/server.hpp>

#include <filesystem>

namespace open_power
{
namespace occ
{

class Manager;

namespace powermode
{
namespace Base = sdbusplus::xyz::openbmc_project::Control::Power::server;
using ModeInterface = sdbusplus::server::object::object<Base::Mode>;
using IpsInterface = sdbusplus::server::object::object<Base::IdlePowerSaver>;
using namespace std::literals::string_literals;

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

const auto PMODE_DEFAULT_INTERFACE =
    "xyz.openbmc_project.Configuration.PowerModeProperties"s;

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

struct PowerModeData
{
    bool modeInitialized = false;
    SysPwrMode mode = SysPwrMode::NO_CHANGE;
    uint16_t oemModeData = 0x0000;
    bool ipsInitialized = false;
    bool ipsEnabled = true;
    uint8_t ipsEnterUtil = 0;
    uint16_t ipsEnterTime = 0;
    uint8_t ipsExitUtil = 0;
    uint16_t ipsExitTime = 0;

    /** @brief Function specifying data to archive for cereal.
     */
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(modeInitialized, mode, oemModeData, ipsInitialized, ipsEnabled,
                ipsEnterUtil, ipsEnterTime, ipsExitUtil, ipsExitTime);
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

    /** @brief Loads any saved power mode data */
    OccPersistData()
    {
        load();
    }

    /** @brief Save Power Mode data to persistent file
     *
     *  @param[in] newMode - desired System Power Mode
     *  @param[in] oemModeData - data required by some OEM Power Modes
     */
    void updateMode(const SysPwrMode newMode, const uint16_t oemModeData)
    {
        modeData.mode = newMode;
        modeData.oemModeData = oemModeData;
        modeData.modeInitialized = true;
        save();
    }

    /** @brief Write Idle Power Saver parameters to persistent file
     *
     *  @param[in] enabled - Idle Power Save status (true = enabled)
     *  @param[in] enterUtil - IPS Enter Utilization (%)
     *  @param[in] enterTime - IPS Enter Time (seconds)
     *  @param[in] exitUtil - IPS Exit Utilization (%)
     *  @param[in] exitTime - IPS Exit Time (seconds)
     */
    void updateIPS(const bool enabled, const uint8_t enterUtil,
                   const uint16_t enterTime, const uint8_t exitUtil,
                   const uint16_t exitTime)
    {
        modeData.ipsEnabled = enabled;
        modeData.ipsEnterUtil = enterUtil;
        modeData.ipsEnterTime = enterTime;
        modeData.ipsExitUtil = exitUtil;
        modeData.ipsExitTime = exitTime;
        modeData.ipsInitialized = true;
        save();
    }

    /** @brief Return the Power Mode and mode data
     *
     *  @param[out] mode - current system power mode
     *  @param[out] oemModeData - frequency data for some OEM mode
     *
     *  @returns true if mode was available
     */
    bool getMode(SysPwrMode& mode, uint16_t& oemModeData) const
    {
        if (!modeData.modeInitialized)
        {
            return false;
        }

        mode = modeData.mode;
        oemModeData = modeData.oemModeData;
        return true;
    }

    /** @brief Get the Idle Power Saver properties from DBus
     *
     *  @param[out] enabled - Idle Power Save status (true = enabled)
     *  @param[out] enterUtil - IPS Enter Utilization (%)
     *  @param[out] enterTime - IPS Enter Time (seconds)
     *  @param[out] exitUtil - IPS Exit Utilization (%)
     *  @param[out] exitTime - IPS Exit Time (seconds)
     *
     * @return true if parameters were read successfully
     */
    bool getIPS(bool& enabled, uint8_t& enterUtil, uint16_t& enterTime,
                uint8_t& exitUtil, uint16_t& exitTime)
    {
        if (!modeData.ipsInitialized)
        {
            return false;
        }

        enabled = modeData.ipsEnabled;
        enterUtil = modeData.ipsEnterUtil;
        enterTime = modeData.ipsEnterTime;
        exitUtil = modeData.ipsExitUtil;
        exitTime = modeData.ipsExitTime;
        return true;
    }

    /** @brief Return true if the power mode is available */
    bool modeAvailable()
    {
        return (modeData.modeInitialized);
    }

    /** @brief Return true if the power mode is available */
    bool ipsAvailable()
    {
        return (modeData.ipsInitialized);
    }

    /** @brief Saves the Power Mode data in the filesystem using cereal. */
    void save();

    /** @brief Trace the Power Mode and IPS parameters. */
    void print();

  private:
    /** @brief Power Mode data filename to store persistent data */
    static constexpr auto powerModeFilename = "powerModeData";

    /** @brief Power Mode data object to be persisted */
    PowerModeData modeData;

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

class PowerMode : public ModeInterface, public IpsInterface
{
  public:
    /** @brief PowerMode object to inform occ of changes to mode
     *
     * This object will monitor for changes to the power mode setting.
     * If a change is detected, and the occ is active, then this object will
     * notify the OCC of the change.
     *
     * @param[in] managerRef - manager object reference
     * @param[in] modePath - Power Mode dbus path
     * @param[in] ipsPath - Idle Power Saver dbus path
     */
    explicit PowerMode(const Manager& managerRef, const char* modePath,
                       const char* ipsPath) :
        ModeInterface(utils::getBus(), modePath, false),
        IpsInterface(utils::getBus(), ipsPath, false), manager(managerRef),
        pmodeMatch(utils::getBus(),
                   sdbusplus::bus::match::rules::propertiesChanged(
                       PMODE_PATH, PMODE_INTERFACE),
                   [this](auto& msg) { this->modeChanged(msg); }),
        ipsMatch(utils::getBus(),
                 sdbusplus::bus::match::rules::propertiesChanged(
                     PIPS_PATH, PIPS_INTERFACE),
                 [this](auto& msg) { this->ipsChanged(msg); }),
        defaultsUpdateMatch(
            utils::getBus(),
            sdbusplus::bus::match::rules::propertiesChangedNamespace(
                "/xyz/openbmc_project/inventory", PMODE_DEFAULT_INTERFACE),
            [this](auto& msg) { this->defaultsReady(msg); }),
        masterOccSet(false), masterActive(false)
    {
        // restore Power Mode to DBus
        SysPwrMode currentMode;
        uint16_t oemModeData = 0;
        if (getMode(currentMode, oemModeData))
        {
            updateDbusMode(currentMode);
        }
        // restore Idle Power Saver parameters to DBus
        uint8_t enterUtil, exitUtil;
        uint16_t enterTime, exitTime;
        bool ipsEnabled;
        if (getIPSParms(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime))
        {
            updateDbusIPS(ipsEnabled, enterUtil, enterTime, exitUtil, exitTime);
        }
    };

    /** @brief Initialize the persistent data with default values
     *
     * @return true if initialization completed
     */
    bool initPersistentData();

    /** @brief Set the current power mode property
     *
     * @param[in] newMode     - desired system power mode
     * @param[in] oemModeData - data required by some OEM Power Modes
     *
     * @return true if mode accepted
     */
    bool setMode(const SysPwrMode newMode, const uint16_t oemModeData);

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

    /** @brief Used to subscribe to dbus defaults property changes **/
    sdbusplus::bus::match_t defaultsUpdateMatch;

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

    /** @brief Get the current power mode property
     *
     * @param[out] currentMode - current system power mode
     * @param[out] oemModeData - frequency data for some OEM mode
     *
     * @return true if data read successfully
     */
    bool getMode(SysPwrMode& currentMode, uint16_t& oemModeData);

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

    /** @brief Get the Idle Power Saver properties
     *
     *  @param[out] enabled - Idle Power Save status (true = enabled)
     *  @param[out] enterUtil - IPS Enter Utilization (%)
     *  @param[out] enterTime - IPS Enter Time (seconds)
     *  @param[out] exitUtil - IPS Exit Utilization (%)
     *  @param[out] exitTime - IPS Exit Time (seconds)
     *
     * @return true if data read successfully
     */
    bool getIPSParms(bool& enabled, uint8_t& enterUtil, uint16_t& enterTime,
                     uint8_t& exitUtil, uint16_t& exitTime);

    /** Update the Idle Power Saver data on DBus
     *
     *  @param[in] enabled - Idle Power Save status (true = enabled)
     *  @param[in] enterUtil - IPS Enter Utilization (%)
     *  @param[in] enterTime - IPS Enter Time (seconds)
     *  @param[in] exitUtil - IPS Exit Utilization (%)
     *  @param[in] exitTime - IPS Exit Time (seconds)
     *
     *  @return true if parameters were set successfully
     */
    bool updateDbusIPS(const bool enabled, const uint8_t enterUtil,
                       const uint16_t enterTime, const uint8_t exitUtil,
                       const uint16_t exitTime);

    /** @brief Callback for entity manager default changes
     *
     * Called when PowerModeProperties defaults are available
     */
    void defaultsReady(sdbusplus::message::message& msg);

    /** @brief Get the default power mode property for this system type
     *
     * @param[out] defaultMode - default system power mode
     *
     * @return true if data read successfully
     */
    bool getDefaultMode(SysPwrMode& defaultMode);

    /** @brief Get the default Idle Power Saver properties for this system type
     *
     *  @param[out] enabled - Idle Power Save status (true = enabled)
     *  @param[out] enterUtil - IPS Enter Utilization (%)
     *  @param[out] enterTime - IPS Enter Time (seconds)
     *  @param[out] exitUtil - IPS Exit Utilization (%)
     *  @param[out] exitTime - IPS Exit Time (seconds)
     *
     *  @return true if parameters were read successfully
     */
    bool getDefaultIPSParms(bool& enabled, uint8_t& enterUtil,
                            uint16_t& enterTime, uint8_t& exitUtil,
                            uint16_t& exitTime);
};

} // namespace powermode

} // namespace occ

} // namespace open_power
#endif
