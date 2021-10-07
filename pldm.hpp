#pragma once

#include "occ_status.hpp"
#include "utils.hpp"

#include <libpldm/pldm.h>

#include <sdbusplus/bus/match.hpp>

namespace pldm
{

namespace MatchRules = sdbusplus::bus::match::rules;

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
        std::function<void(open_power::occ::instanceID, bool)> sbeCallBack) :
        callBack(callBack),
        sbeCallBack(sbeCallBack),
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
                      std::placeholders::_1))
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
     *  @param[in] entityId - the entity ID to query
     *  @param[in] stateSetId - the state set ID to look for
     *  @param[out] instanceToEffecterMap - map of instance to effecterID
     *  @param[out] count - sensor offset of interested state set ID
     *  @param[out] stateIdPos - position of the stateSetID
     */
    void fetchEffecterInfo(uint16_t entityId, uint16_t stateSetId,
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
    SensorOffset OCCSensorOffset;

    /** @brief Sensor offset of the SBE state set ID
     * PLDM_OEM_IBM_SBE_HRESET_STATE in state sensor PDR.
     */
    SensorOffset SBESensorOffset;

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

    /** @brief Send the PLDM request
     *
     * @param[in] - the request data
     */
    void sendPldm(const std::vector<uint8_t>& request);
};

} // namespace pldm
