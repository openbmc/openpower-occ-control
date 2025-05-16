#pragma once

#include "legacy/occ_pass_through_legacy.hpp"
#include "legacy/occ_status_legacy.hpp"


#include "powercap.hpp"
#include "utils.hpp"

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

constexpr unsigned int defaultPollingInterval = 1;

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
    explicit Manager(EventPtr& event) :
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

    }

    /** @brief Return the number of bound OCCs */
    inline auto getNumOCCs() const
    {
        return activeCount;
    }

    /** @brief Return the latest ambient and altitude readings
     *
     *  @param[out] ambientValid - true if ambientTemp is valid
     *  @param[out] ambient - ambient temperature in degrees C
     *  @param[out] altitude - altitude in meters
     */
    void getAmbientData(bool& ambientValid, uint8_t& ambientTemp,
                        uint16_t& altitude) const;

    /** @brief Notify pcap object to update bounds */
    void updatePcapBounds() const;

    /**
     * @brief Set all sensor values of this OCC to NaN.
     * @param[in] id - Id of the OCC.
     * */
    void setSensorValueToNaN(uint32_t id) const;

    /** @brief Set all sensor values of this OCC to NaN and non functional.
     *
     *  @param[in] id - Id of the OCC.
     */
    void setSensorValueToNonFunctional(uint32_t id) const;

    /** @brief Clear any state flags that need to be reset when the host state
     * is off */
    void hostPoweredOff();

  private:
    /** @brief Creates the OCC D-Bus objects.
     */
    void findAndCreateObjects();

    /** @brief Callback that responds to cpu creation in the inventory -
     *         by creating the needed objects.
     *
     *  @param[in] msg - bus message
     *
     *  @returns 0 to indicate success
     */
    int cpuCreated(sdbusplus::message_t& msg);

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
    void statusCallBack(instanceID instance, bool status);

    /** @brief Set flag that a PM Complex reset is needed (to be initiated
     * later) */
    void resetOccRequest(instanceID instance);

    /** @brief Initiate the request to reset the PM Complex (PLDM -> HBRT) */
    void initiateOccRequest(instanceID instance);

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
    void initTimerObjects();

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

    /** @brief Read the altitude from DBus */
    void readAltitude();

    /** @brief Callback function when ambient temperature changes
     *
     *  @param[in]  msg - Data associated with subscribed signal
     */
    void ambientCallback(sdbusplus::message_t& msg);

    /** @brief Confirm that a single OCC master was found and start presence
     * monitoring
     */
    void validateOccMaster();
};

} // namespace occ
} // namespace open_power
