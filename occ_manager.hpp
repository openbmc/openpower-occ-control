#pragma once

#include "occ_pass_through.hpp"
#include "occ_status.hpp"
#ifdef PLDM
#include "pldm.hpp"
#endif
#include "powercap.hpp"
#include "utils.hpp"
#ifdef POWER10
#include "powermode.hpp"
#endif

#include <cstring>
#include <functional>
#include <sdbusplus/bus.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>
#include <vector>

namespace sdbusRule = sdbusplus::bus::match::rules;
namespace open_power
{
namespace occ
{

#ifdef READ_OCC_SENSORS
enum occFruType
{
    processorCore = 0,
    internalMemCtlr = 1,
    dimm = 2,
    memCtrlAndDimm = 3,
    VRMVdd = 6,
    PMIC = 7,
    memCtlrExSensor = 8
};
#endif

/** @brief Default time, in seconds, between OCC poll commands */
constexpr unsigned int defaultPollingInterval = 1;

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
    Manager(EventPtr& event) :
        event(event), pollInterval(defaultPollingInterval),
        sdpEvent(sdeventplus::Event::get_default()),
        _pollTimer(
            std::make_unique<
                sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>(
                sdpEvent, std::bind(&Manager::pollerTimerExpired, this)))
#ifdef PLDM
        ,
        pldmHandle(std::make_unique<pldm::Interface>(
            std::bind(std::mem_fn(&Manager::updateOCCActive), this,
                      std::placeholders::_1, std::placeholders::_2)))
#endif

    {
#ifdef I2C_OCC
        // I2C OCC status objects are initialized directly
        initStatusObjects();
#else
        findAndCreateObjects();
#endif
    }

    /** @brief Return the number of bound OCCs */
    inline auto getNumOCCs() const
    {
        return activeCount;
    }

  private:
    /** @brief Checks if the CPU inventory is present and if so, creates
     *         the occ D-Bus objects. Else, registers a handler to be
     *         called when inventory is created.
     */
    void findAndCreateObjects();

    /** @brief Callback that responds to cpu creation in the inventory -
     *         by creating the needed objects.
     *
     *  @param[in] msg - bus message
     *
     *  @returns 0 to indicate success
     */
    int cpuCreated(sdbusplus::message::message& msg);

    /** @brief Create child OCC objects.
     *
     *  @param[in] occ - the occ name, such as occ0.
     */
    void createObjects(const std::string& occ);

    /** @brief Callback handler invoked by Status object when the OccActive
     *         property is changed. This is needed to make sure that the
     *         error detection is started only after all the OCCs are bound.
     *         Similarly, when one of the OCC gets its OccActive property
     *         un-set, then the OCC error detection needs to be stopped on
     *         all the OCCs
     *
     *  @param[in] status - OccActive status
     */
    void statusCallBack(bool status);

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

#ifdef POWER10
    /** @brief Power mode monitor and notification object */
    std::unique_ptr<open_power::occ::powermode::PowerMode> pmode;
#endif

    /** @brief sbdbusplus match objects */
    std::vector<sdbusplus::bus::match_t> cpuMatches;

    /** @brief Number of OCCs that are bound */
    uint8_t activeCount = 0;

    /** @brief Number of seconds between poll commands */
    uint8_t pollInterval;

    /** @brief Poll timer event */
    sdeventplus::Event sdpEvent;

    /**
     * @brief The timer to be used once the OCC goes active.  When it expires,
     *        a POLL command will be sent to the OCC and then timer restarted.
     */
    std::unique_ptr<
        sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>>
        _pollTimer;

#ifdef I2C_OCC
    /** @brief Init Status objects for I2C OCC devices
     *
     * It iterates in /sys/bus/i2c/devices, finds all occ hwmon devices
     * and creates status objects.
     */
    void initStatusObjects();
#endif

#ifdef PLDM
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
    bool updateOCCActive(instanceID instance, bool status);

    std::unique_ptr<pldm::Interface> pldmHandle = nullptr;
#endif

    /**
     * @brief Called when poll timer expires and forces a POLL command to the
     * OCC. The poll timer will then be restarted.
     * */
    void pollerTimerExpired();

    /**
     * @brief Finds the OCC devices in /dev
     *
     * @return The IDs of the OCCs - 0, 1, etc.
     */
    std::vector<int> findOCCsInDev();

#ifdef READ_OCC_SENSORS
    /**
     * @brief Gets the occ sensor values.
     * @param[in] id - Id of the OCC.
     * @param[in] masterOcc - Is this OCC the master OCC.
     * */
    void getSensorValues(uint32_t id, bool masterOcc);

    /**
     * @brief Trigger OCC driver to read the temperature sensors.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void readTempSensors(const fs::path& path, uint32_t id);

    /**
     * @brief Trigger OCC driver to read the power sensors.
     * @param[in] path - path of the OCC sensors.
     * @param[in] id - Id of the OCC.
     * */
    void readPowerSensors(const fs::path& path, uint32_t id);

    /**
     * @brief Set all sensor values of this OCC to NaN.
     * @param[in] id - Id of the OCC.
     * */
    void setSensorValueToNaN(uint32_t id);

    /** @brief Store the existing OCC sensors on D-BUS */
    std::map<std::string, uint32_t> existingSensors;

    /** @brief Get FunctionID from the `powerX_label` file.
     *  @param[in] value - the value of the `powerX_label` file.
     *  @returns FunctionID of the power sensors.
     */
    std::optional<std::string>
        getPowerLabelFunctionID(const std::string& value);

    /** @brief The power sensor names map */
    const std::map<std::string, std::string> powerSensorName = {
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
        {"26", "p0_mem_1_power"},  {"27", "p0_mem_2_power"}};

    /** @brief The dimm temperature sensor names map */
    const std::map<uint32_t, std::string> dimmTempSensorName = {
        {internalMemCtlr, "_intmb_temp"},
        {dimm, "_dram_temp"},
        {memCtrlAndDimm, "_dram_extmb_temp"},
        {PMIC, "_pmic_temp"},
        {memCtlrExSensor, "_extmb_temp"}};
#endif
};

} // namespace occ
} // namespace open_power
