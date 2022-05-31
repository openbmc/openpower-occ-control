#pragma once
#include "occ_events.hpp"
#include "occ_status.hpp"
#include "utils.hpp"

#include <libpldm/pldm.h>

#include <sdbusplus/bus/match.hpp>
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>

namespace pldm
{

namespace MatchRules = sdbusplus::bus::match::rules;
using namespace open_power::occ;

using CompositeEffecterCount = uint8_t;
using EffecterID = uint16_t;
using EntityType = uint16_t;
using EntityInstance = uint16_t;
using EventState = uint8_t;
using InstanceToEffecter = std::map<open_power::occ::instanceID, EffecterID>;
using PdrList = std::vector<std::vector<uint8_t>>;
using SensorID = uint16_t;
using SensorOffset = uint8_t;
using SensorToInstance = std::map<SensorID, open_power::occ::instanceID>;
using TerminusID = uint8_t;

/** @brief Hardcoded TID */
constexpr TerminusID tid = 0;

/** @brief OCC instance starts with 0 for example "occ0" */
constexpr open_power::occ::instanceID start = 0;

/** @brief Hardcoded mctpEid for HBRT */
constexpr mctp_eid_t mctpEid = 10;

/** @class Interface
 *
 *  @brief Abstracts the PLDM details related to the OCC
 */
class Interface
{
  public:
    Interface() = delete;
    ~Interface() = default;
    Interface(const Interface&) = delete;
    Interface& operator=(const Interface&) = delete;
    Interface(Interface&&) = delete;
    Interface& operator=(Interface&&) = delete;

    /** @brief Constructs the PLDM Interface object for OCC functions
     *
     *  @param[in] callBack - callBack handler to invoke when the OCC state
     *                        changes.
     */
    explicit Interface(
        std::function<bool(open_power::occ::instanceID, bool)> callBack,
        std::function<void(open_power::occ::instanceID, bool)> sbeCallBack,
        std::function<void(bool)> safeModeCallBack, EventPtr& event) :
        callBack(callBack),
        sbeCallBack(sbeCallBack), safeModeCallBack(safeModeCallBack),
        event(event),
        pldmEventSignal(
            open_power::occ::utils::getBus(),
            MatchRules::type::signal() +
                MatchRules::member("StateSensorEvent") +
                MatchRules::path("/xyz/openbmc_project/pldm") +
                MatchRules::interface("xyz.openbmc_project.PLDM.Event"),
            std::bind(std::mem_fn(&Interface::sensorEvent), this,
                      std::placeholders::_1)),
        hostStateSignal(
            open_power::occ::utils::getBus(),
            MatchRules::propertiesChanged("/xyz/openbmc_project/state/host0",
                                          "xyz.openbmc_project.State.Host"),
            std::bind(std::mem_fn(&Interface::hostStateEvent), this,
                      std::placeholders::_1)),
        sdpEvent(sdeventplus::Event::get_default()),
        pldmRspTimer(
            sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>(
                sdpEvent, std::bind(&Interface::pldmRspExpired, this)))
    {}

    /** @brief Fetch the state sensor PDRs and populate the cache with
     *         sensorId to OCC/SBE instance mapping information and the sensor
     *         offset for the relevent state set.
     *
     *  @param[in] stateSetId - the state set ID to look for
     *  @param[out] sensorInstanceMap - map of sensorID to instance
     *  @param[out] sensorOffset - sensor offset of interested state set ID
     */
    void fetchSensorInfo(uint16_t stateSetId,
                         SensorToInstance& sensorInstanceMap,
                         SensorOffset& sensorOffset);

    /** @brief Fetch the OCC/SBE state effecter PDRs and populate the cache
     *         with OCC/SBE instance to EffecterID information.
     *
     *  @param[in] stateSetId - the state set ID to look for
     *  @param[out] instanceToEffecterMap - map of instance to effecterID
     *  @param[out] count - sensor offset of interested state set ID
     *  @param[out] stateIdPos - position of the stateSetID
     */
    void fetchEffecterInfo(uint16_t stateSetId,
                           InstanceToEffecter& instanceToEffecterMap,
                           CompositeEffecterCount& count, uint8_t& stateIdPos);

    /** @brief Prepare the request for SetStateEffecterStates command
     *
     *  @param[in] instanceId - PLDM instanceID
     *  @param[in] effecterId - the instance effecter ID
     *  @param[in] effecterCount - compositeEffecterCount for the effecter PDR
     *  @param[in] stateIdPos - position of the stateSetID
     *  @param[in] stateSetValue - the value to set the state set to
     *
     *  @return PLDM request message to be sent to host for OCC reset or SBE
     *          HRESET, empty response in the case of failure.
     */
    std::vector<uint8_t>
        prepareSetEffecterReq(uint8_t instanceId, EffecterID effecterId,
                              CompositeEffecterCount effecterCount,
                              uint8_t stateIdPos, uint8_t stateSetValue);

    /** @brief Send the PLDM message to reset the OCC
     *
     *  @param[in] instanceId - OCC instance to reset
     *
     */
    void resetOCC(open_power::occ::instanceID occInstanceId);

    /** @brief Send the PLDM message to perform the HRESET
     *
     *  @param[in] instanceID - SBE instance to HRESET
     */
    void sendHRESET(open_power::occ::instanceID sbeInstanceId);

    /** @brief Check if the OCC active sensor is available
     *         On successful read, the Manager callback will be called to update
     *         the status
     *
     *  @param[in] instance  - OCC instance to check
     */
    void checkActiveSensor(uint8_t instance);

  private:
    /** @brief Callback handler to be invoked when the state of the OCC
     *         changes
     */
    std::function<bool(open_power::occ::instanceID, bool)> callBack = nullptr;

    /** @brief Callback handler to be invoked when the maintenance state of the
     *         SBE changes
     */
    std::function<void(open_power::occ::instanceID, bool)> sbeCallBack =
        nullptr;

    /** @brief Callback handler to be invoked when the OCC is in SAFE Mode =
     *         true or when OCCs are in_service = false.
     */
    std::function<void(bool)> safeModeCallBack = nullptr;

    /** @brief reference to sd_event wrapped in unique_ptr */
    EventPtr& event;

    /** @brief event source wrapped in unique_ptr */
    EventSourcePtr eventSource;

    /** @brief Used to subscribe to D-Bus PLDM StateSensorEvent signal and
     *         processes if the event corresponds to OCC state change.
     */
    sdbusplus::bus::match_t pldmEventSignal;

    /** @brief Used to subscribe for host state change signal */
    sdbusplus::bus::match_t hostStateSignal;

    /** @brief PLDM Sensor ID to OCC Instance mapping
     */
    SensorToInstance sensorToOCCInstance;

    /** @brief PLDM Sensor ID to SBE Instance mapping
     */
    SensorToInstance sensorToSBEInstance;

    /** @brief Sensor offset of OCC state set ID
     * PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS in state sensor PDR.
     */
    SensorOffset OCCSensorOffset = 0;

    /** @brief Sensor offset of the SBE state set ID
     * PLDM_OEM_IBM_SBE_HRESET_STATE in state sensor PDR.
     */
    SensorOffset SBESensorOffset = 0;

    /** @brief OCC Instance mapping to PLDM Effecter ID
     */
    InstanceToEffecter occInstanceToEffecter;

    /** @brief SBE instance mapping to PLDM Effecter ID
     */
    InstanceToEffecter sbeInstanceToEffecter;

    /** @brief compositeEffecterCount for OCC reset state effecter PDR */
    CompositeEffecterCount OCCEffecterCount = 0;

    /** @brief compositeEffecterCount for SBE HRESET state effecter PDR */
    CompositeEffecterCount SBEEffecterCount = 0;

    /** @brief Position of Boot/Restart Cause stateSetID in OCC state
     *         effecter PDR
     */
    uint8_t bootRestartPosition = 0;

    /** @brief Position of the SBE maintenance stateSetID in the state
     *         effecter PDR
     */
    uint8_t sbeMaintenanceStatePosition = 0;

    /** @brief OCC instance number for the PLDM message */
    uint8_t pldmResponseOcc = 0;

    /** @brief File descriptor for PLDM messages */
    int pldmFd = -1;

    /** @brief MCTP instance number used in PLDM requests
     */
    uint8_t mctpInstance{};

    /** @brief The response for the PLDM request msg is received flag.
     */
    bool pldmResponseReceived = false;

    /** @brief The response for the PLDM request has timed out.
     */
    bool pldmResponseTimeout = false;

    /** @brief timer event */
    sdeventplus::Event sdpEvent;

    /** @brief Timer that is started when PLDM command is sent
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic> pldmRspTimer;

    std::set<uint8_t> outstandingHResets;

    /** @brief Callback when PLDM response has not been received within the
     * timeout period.
     */
    void pldmRspExpired();

    /** @brief Close the MCTP file */
    void pldmClose();

    /** @brief When the OCC state changes host sends PlatformEventMessage
     *         StateSensorEvent, this function processes the D-Bus signal
     *         with the sensor event information and invokes the callback
     *         to change the OCC state.
     *
     *  @param[in] msg - data associated with the subscribed signal
     */
    void sensorEvent(sdbusplus::message::message& msg);

    /** @brief When the host state changes and if the CurrentHostState is
     *         xyz.openbmc_project.State.Host.HostState.Off then
     *         the cache of OCC sensors and effecters mapping is cleared.
     *
     *  @param[in] msg - data associated with the subscribed signal
     */
    void hostStateEvent(sdbusplus::message::message& msg);

    /** @brief Called when it is determined that the Host is not running.
     *         The cache of OCC sensors and effecters mapping is cleared.
     */
    void clearData();

    /** @brief Check if the PDR cache for PLDM OCC sensors is valid
     *
     *  @return true if cache is populated and false if the cache is not
     *          populated.
     */
    auto isOCCSensorCacheValid()
    {
        return (sensorToOCCInstance.empty() ? false : true);
    }

    /** @brief Check if the PDR cache for PLDM OCC effecters is valid
     *
     *  @return true if cache is populated and false if the cache is not
     *          populated.
     */
    auto isPDREffecterCacheValid()
    {
        return (occInstanceToEffecter.empty() ? false : true);
    }

    /** @brief Query PLDM for the MCTP requester instance id
     *
     * @param[out] - the instance id
     *
     * @return true if the id was found and false if not
     */
    bool getMctpInstanceId(uint8_t& instanceId);

    /** @brief Encode a GetStateSensor command into a PLDM request
     *  @param[in] instance - OCC instance number
     *  @param[in] sensorId - OCC Active sensor ID number
     *
     *  @return request - The encoded PLDM messsage to be sent
     */
    std::vector<uint8_t> encodeGetStateSensorRequest(uint8_t instance,
                                                     uint16_t sensorId);
    /** @brief Send the PLDM request
     *
     * @param[in] request - the request data
     * @param[in] rspExpected - false: no need to wait for the response
     *                          true: will need to process response in callback
     */
    void sendPldm(const std::vector<uint8_t>& request, const uint8_t instance,
                  const bool rspExpected = false);

    /** @brief Register a callback function to handle the PLDM response */
    void registerPldmRspCallback();

    /** @brief callback for the PLDM response event
     *
     *  @param[in] es       - Populated event source
     *  @param[in] fd       - Associated File descriptor
     *  @param[in] revents  - Type of event
     *  @param[in] userData - User data that was passed during registration
     *
     *  @return             - 0 or positive number on success and negative
     *                        errno otherwise
     */
    static int pldmRspCallback(sd_event_source* es, int fd, uint32_t revents,
                               void* userData);
};

} // namespace pldm
