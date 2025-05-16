#pragma once

#include "occ_pass_through.hpp" //SHELDON: PUT BACK
#include "occ_status.hpp"
#include "powercap.hpp"
#include "utils.hpp"

#include "pldm.hpp"

#ifdef PHAL_SUPPORT
#include <libphal.H>
#endif // PHAL_SUPPORT

#include "powermode.hpp"

#include <sdbusplus/bus.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>

#include <cstring>
#include <functional>
#include <vector>

namespace sdbusRule = sdbusplus::bus::match::rules;
namespace open_power
{
namespace occ
{

enum occFruType
{
    processorCore = 0,
    internalMemCtlr = 1,
    dimm = 2,
    memCtrlAndDimm = 3,
    VRMVdd = 6,
    PMIC = 7,
    memCtlrExSensor = 8,
    processorIoRing = 9
};

/** @brief Default time, in seconds, between OCC poll commands */
constexpr unsigned int defaultPollingInterval = 5;

/** @brief Default time, in seconds, between OCC poll commands */

constexpr auto AMBIENT_PATH =
    "/xyz/openbmc_project/sensors/temperature/Ambient_Virtual_Temp";
constexpr auto AMBIENT_INTERFACE = "xyz.openbmc_project.Sensor.Value";
constexpr auto AMBIENT_PROP = "Value";
constexpr auto ALTITUDE_PATH = "/xyz/openbmc_project/sensors/altitude/Altitude";
constexpr auto ALTITUDE_INTERFACE = "xyz.openbmc_project.Sensor.Value";
constexpr auto ALTITUDE_PROP = "Value";

constexpr auto EXTN_LABEL_PWRM_MEMORY_POWER = "5057524d";
constexpr auto EXTN_LABEL_PWRP_PROCESSOR_POWER = "50575250";

/** @class Manager
 *  @brief Builds and manages OCC objects
 */
struct Manager
{
  public:
    Manager() = delete;
    Manager(const Manager&) = delete;
    Manager& operator=(const Manager&) = delete;
    Manager(Manager&&) = delete;
    Manager& operator=(Manager&&) = delete;
    ~Manager() = default;

    /** @brief Adds OCC pass-through and status objects on the bus
     *         when corresponding CPU inventory is created.
     *
     *  @param[in] event - Unique ptr reference to sd_event
     */
    explicit Manager(EventPtr& event) :                                         // SHELDON: BASE, Inherited P10-11 ++
        event(event), pollInterval(defaultPollingInterval),
        sdpEvent(sdeventplus::Event::get_default()),
        ambientPropChanged(
            utils::getBus(),
            sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(AMBIENT_PATH) +
                sdbusRule::argN(0, AMBIENT_INTERFACE) +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
            std::bind(&Manager::ambientCallback, this, std::placeholders::_1))
    {

        initTimerObjects();

        findAndCreateObjects();

        readAltitude();

        createPldmHandle();

    }

    /** @brief Return the number of bound OCCs */
    inline auto getNumOCCs() const                                              // SHELDON: BASE
    {
        return activeCount;
    }

    /** @brief Called by a Device to report that the SBE timed out
     *         and appropriate action should be taken
     *
     * @param[in] instance - the OCC instance id
     */
    void sbeTimeout(unsigned int instance);                                     // SHELDON: NEW P10-11 ++

    /** @brief Return the latest ambient and altitude readings
     *
     *  @param[out] ambientValid - true if ambientTemp is valid
     *  @param[out] ambient - ambient temperature in degrees C
     *  @param[out] altitude - altitude in meters
     */
    void getAmbientData(bool& ambientValid, uint8_t& ambientTemp,               // SHELDON: BASE
                        uint16_t& altitude) const;

    /** @brief Notify pcap object to update bounds */
    void updatePcapBounds() const;                                              // SHELDON: BASE

    /**
     * @brief Set all sensor values of this OCC to NaN.
     * @param[in] id - Id of the OCC.
     * */
    void setSensorValueToNaN(uint32_t id) const;                                // SHELDON: NEW P10-11 ++

    /** @brief Set all sensor values of this OCC to NaN and non functional.
     *
     *  @param[in] id - Id of the OCC.
     */
    void setSensorValueToNonFunctional(uint32_t id) const;                      // SHELDON: NEW P10-11 ++

    /** @brief Clear any state flags that need to be reset when the host state
     * is off */
    void hostPoweredOff();                                                      // SHELDON: BASE

  private:
    /** @brief Creates the OCC D-Bus objects.
     */
    void findAndCreateObjects();                                                // SHELDON: Inherited to P8,P9,P10-11,P12?

    /** @brief Callback that responds to cpu creation in the inventory -
     *         by creating the needed objects.
     *
     *  @param[in] msg - bus message
     *
     *  @returns 0 to indicate success
     */
    int cpuCreated(sdbusplus::message_t& msg);                                  // SHELDON: BASE

    /** @brief Create child OCC objects.
     *
     *  @param[in] occ - the occ name, such as occ0.
     */
    void createObjects(const std::string& occ);                                 // SHELDON: BASE, Inherited P10-11 ++

    /** @brief Callback handler invoked by Status object when the OccActive
     *         property is changed. This is needed to make sure that the
     *         error detection is started only after all the OCCs are bound.
     *         Similarly, when one of the OCC gets its OccActive property
     *         un-set, then the OCC error detection needs to be stopped on
     *         all the OCCs
     *
     *  @param[in] status - OccActive status
     */
    void statusCallBack(instanceID instance, bool status);                      // SHELDON: BASE Inherited P10-11 ++

    /** @brief Set flag that a PM Complex reset is needed (to be initiated
     * later) */
    void resetOccRequest(instanceID instance);                                  // SHELDON: BASE

    /** @brief Initiate the request to reset the PM Complex (PLDM -> HBRT) */
    void initiateOccRequest(instanceID instance);                               // SHELDON: BASE Inherited P10-11 ++

    /** @brief Sends a Heartbeat command to host control command handler */
    void sendHeartBeat();

    /** @brief reference to sd_event wrapped in unique_ptr */
    EventPtr& event;

    /** @brief OCC pass-through objects */
    std::vector<std::unique_ptr<PassThrough>> passThroughObjects;

    /** @brief OCC Status objects */
    std::vector<std::unique_ptr<Status>> statusObjects;

    /** @brief Power cap monitor and occ notification object */
    std::unique_ptr<open_power::occ::powercap::PowerCap> pcap;

    /** @brief sbdbusplus match objects */
    std::vector<sdbusplus::bus::match_t> cpuMatches;

    /** @brief Number of OCCs that are bound */
    uint8_t activeCount = 0;

    /** @brief Number of seconds between poll commands */
    uint8_t pollInterval;

    /** @brief Ambient temperature of the system in degrees C */
    uint8_t ambient = 0xFF; // default: not available

    /** @brief Altitude of the system in meters */
    uint16_t altitude = 0xFFFF; // default: not available

    /** @brief Poll timer event */
    sdeventplus::Event sdpEvent;

    /** @brief Flags to indicate if waiting for all of the OCC active sensors to
     * come online */
    bool waitingForAllOccActiveSensors = false;

    /** @brief Set containing intance numbers of any OCCs that became active
     *         while waiting for status objects to be created */
    std::set<uint8_t> queuedActiveState;

    /**
     * @brief The timer to be used once the OCC goes active.  When it expires,
     *        a POLL command will be sent to the OCC and then timer restarted.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        _pollTimer;

    /** @brief Subscribe to ambient temperature changed events */
    sdbusplus::bus::match_t ambientPropChanged;

    /** @brief Flag to indicate that a PM complex reset needs to happen */
    bool resetRequired = false;
    /** @brief Instance number of the OCC/processor that triggered the reset */
    uint8_t resetInstance = 255;
    /** @brief Set when a PM complex reset has been issued (to prevent multiple
     * requests) */
    bool resetInProgress = false;

    /** @brief Init timer objects
     *
     * It creates timer objects used to get callbacks.
     */
    void initTimerObjects();                                                    // SHELDON: BASE Inherited P10-11 ++

    void createPldmHandle();                                                    // SHELDON: NEW P10-11 ++

    /** @brief Power mode monitor and notification object */
    std::unique_ptr<open_power::occ::powermode::PowerMode> pmode;               // SHELDON: HERE

    /** @brief Callback handler invoked by the PLDM event handler when state of
     *         the OCC is toggled by the host. The caller passes the instance
     *         of the OCC and state of the OCC.
     *
     *  @param[in] instance - instance of the OCC
     *  @param[in] status - true when the OCC goes active and false when the OCC
     *                      goes inactive
     *
     *  @return true if setting the state of OCC is successful and false if it
     *          fails.
     */
    bool updateOCCActive(instanceID instance, bool status);                     // SHELDON: NEW P10-11 ++

    /** @brief Callback handler invoked by the PLDM event handler when mode of
     *         the OCC SAFE MODE is inacted or cleared.
     */
    void updateOccSafeMode(bool safeState);                                     // SHELDON: NEW P10-11 ++

    /** @brief Callback handler invoked by PLDM sensor change when
     *         the HRESET succeeds or fails.
     *
     *  @param[in] instance - the SBE instance id
     *  @param[in] success - true if the HRESET succeeded, otherwise false
     */
    void sbeHRESETResult(instanceID instance, bool success);                    // SHELDON: NEW P10-11 ++    ?? Quesiton the PHAL_SUPPORT on which driver.

#ifdef PHAL_SUPPORT
    /** @brief Helper function to check whether an SBE dump should be collected
     *         now.
     *
     *  @param[in] instance - the SBE instance id
     *
     *  @return true if an SBE dump should be collected and false if not
     */
    bool sbeCanDump(unsigned int instance);                                     // SHELDON: NEW P10-11 ++  ???? PHAL_SUPPORT ONLY

    /** @brief Helper function to set the SBE state through PDBG/PHAL
     *
     * @param[in] instance - instance of the SBE
     * @param[in] state - the state to which the SBE should be set
     *
     */
    void setSBEState(unsigned int instance, enum sbe_state state);              // SHELDON: NEW P10-11 ++  ???? PHAL_SUPPORT ONLY

    /** @brief Helper function to get the SBE instance PDBG processor target
     *
     * @param[in] instance - the SBE instance id
     *
     * @return a pointer to the PDBG target
     */
    struct pdbg_target* getPdbgTarget(unsigned int instance);                   // SHELDON: NEW P10-11 ++  ???? PHAL_SUPPORT ONLY

    /** @brief Whether pdbg_targets_init has been called */
    bool pdbgInitialized = false;
#endif // PHAL_SUPPORT

    std::unique_ptr<pldm::Interface> pldmHandle = nullptr;

    /**
     * @brief Timer used when discovering OCCs in /dev.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        discoverTimer;

    /**
     * @brief Used when discovering /dev/occ objects to know if
     *        any were added since the last check.
     */
    std::vector<int> prevOCCSearch;

    /**
     * @brief Timer used when waiting for OCCs to go active.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        waitForAllOccsTimer;

    /**
     * @brief Timer used to throttle PLDM traces when there are problems
     determining the OCC status via pldm. Used to prevent excessive
     journal traces.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        throttlePldmTraceTimer;
    /**
     * @brief onPldmTimeoutCreatePel flag will be used to indicate if
     *        a PEL should get created when the throttlePldmTraceTimer expires.
     *        The first time the throttlePldmTraceTimer expires, the traces
     *        will be throttled and then the timer gets restarted. The
     *        next time the timer expires, a PEL will get created.
     */
    bool onPldmTimeoutCreatePel = false;

    /** @brief Check if all of the OCC Active sensors are available and if not
     * restart the discoverTimer
     */
    void throttlePldmTraceExpired();                                            // SHELDON: NEW P10-11 ++

    /** @brief Create a PEL when the code is not able to obtain the OCC PDRs
     * via PLDM. This is called when the throttlePldmTraceTimer expires.
     */
    void createPldmSensorPEL();                                                 // SHELDON: NEW P10-11 ++

    /** @brief Called when code times out waiting for all OCCs to be running or
     *         after the app is restarted (Status does not callback into
     * Manager).
     */
    void occsNotAllRunning();                                                   // SHELDON: NEW P10-11 ++

    /** @brief Check if all of the OCC Active sensors are available and if not
     * restart the discoverTimer
     */
    void checkAllActiveSensors();                                               // SHELDON: NEW P10-11 ++

    /**
     * @brief Gets the occ sensor values.
     * @param[in] occ - pointer to OCCs Status object
     * */
    void getSensorValues(std::unique_ptr<Status>& occ);                         // SHELDON: NEW P10-11 ++

    /**
     * @brief Trigger OCC driver to read the temperature sensors.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void readTempSensors(const fs::path& path, uint32_t id);                    // SHELDON: NEW P10-11 ++

    /**
     * @brief Trigger OCC driver to read the extended sensors.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void readExtnSensors(const fs::path& path, uint32_t id);                    // SHELDON: NEW P10-11 ++

    /**
     * @brief Trigger OCC driver to read the power sensors.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void readPowerSensors(const fs::path& path, uint32_t id);                   // SHELDON: NEW P10-11 ++

    /** @brief Store the existing OCC sensors on D-BUS */
    std::map<std::string, uint32_t> existingSensors;                            // SHELDON: NEW P10-11 ++

    /** @brief Get FunctionID from the `powerX_label` file.
     *  @param[in] value - the value of the `powerX_label` file.
     *  @returns FunctionID of the power sensors.
     */
    std::optional<std::string> getPowerLabelFunctionID(                         // SHELDON: NEW P10-11 ++
        const std::string& value);

    /** @brief The power sensor names map */
    const std::map<std::string, std::string> powerSensorName = {                // SHELDON: NEW P10-11 ++
        {"system", "total_power"}, {"1", "p0_mem_power"},
        {"2", "p1_mem_power"},     {"3", "p2_mem_power"},
        {"4", "p3_mem_power"},     {"5", "p0_power"},
        {"6", "p1_power"},         {"7", "p2_power"},
        {"8", "p3_power"},         {"9", "p0_cache_power"},
        {"10", "p1_cache_power"},  {"11", "p2_cache_power"},
        {"12", "p3_cache_power"},  {"13", "io_a_power"},
        {"14", "io_b_power"},      {"15", "io_c_power"},
        {"16", "fans_a_power"},    {"17", "fans_b_power"},
        {"18", "storage_a_power"}, {"19", "storage_b_power"},
        {"23", "mem_cache_power"}, {"25", "p0_mem_0_power"},
        {"26", "p0_mem_1_power"},  {"27", "p0_mem_2_power"},
        {"35", "pcie_dcm0_power"}, {"36", "pcie_dcm1_power"},
        {"37", "pcie_dcm2_power"}, {"38", "pcie_dcm3_power"},
        {"39", "io_dcm0_power"},   {"40", "io_dcm1_power"},
        {"41", "io_dcm2_power"},   {"42", "io_dcm3_power"},
        {"43", "avdd_total_power"}};

    /** @brief The dimm temperature sensor names map  */
    const std::map<uint32_t, std::string> dimmTempSensorName = {                // SHELDON: NEW P10-11 ++
        {internalMemCtlr, "_intmb_temp"},
        {dimm, "_dram_temp"},
        {memCtrlAndDimm, "_dram_extmb_temp"},
        {PMIC, "_pmic_temp"},
        {memCtlrExSensor, "_extmb_temp"}};

    /** @brief The dimm DVFS temperature sensor names map  */
    const std::map<uint32_t, std::string> dimmDVFSSensorName = {                // SHELDON: NEW P10-11 ++
        {internalMemCtlr, "dimm_intmb_dvfs_temp"},
        {dimm, "dimm_dram_dvfs_temp"},
        {memCtrlAndDimm, "dimm_dram_extmb_dvfs_temp"},
        {PMIC, "dimm_pmic_dvfs_temp"},
        {memCtlrExSensor, "dimm_extmb_dvfs_temp"}};

    /**
     * @brief Called when poll timer expires and forces a POLL command to the
     * OCC. The poll timer will then be restarted.
     * */
    void pollerTimerExpired();                                                  // SHELDON: NEW P10-11 ++

    /**
     * @brief Finds the OCC devices in /dev
     *
     * @return The IDs of the OCCs - 0, 1, etc.
     */
    std::vector<int> findOCCsInDev();                                           // SHELDON: BASE

    /** @brief Read the altitude from DBus */
    void readAltitude();                                                        // SHELDON: BASE

    /** @brief Callback function when ambient temperature changes
     *
     *  @param[in]  msg - Data associated with subscribed signal
     */
    void ambientCallback(sdbusplus::message_t& msg);                            // SHELDON: BASE, Inherited P10-11 ++

    /** @brief Confirm that a single OCC master was found and start presence
     * monitoring
     */
    void validateOccMaster();                                                   // SHELDON: BASE, Inherited P10-11 ++
};

} // namespace occ
} // namespace open_power
