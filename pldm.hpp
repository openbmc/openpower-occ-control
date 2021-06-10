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
using OccInstanceToEffecter = std::map<open_power::occ::instanceID, EffecterID>;
using PdrList = std::vector<std::vector<uint8_t>>;
using SensorID = uint16_t;
using SensorOffset = uint8_t;
using SensorToOCCInstance = std::map<SensorID, open_power::occ::instanceID>;
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
        std::function<bool(open_power::occ::instanceID, bool)> callBack) :
        callBack(callBack),
        pldmEventSignal(
            utils::getBus(),
            MatchRules::type::signal() +
                MatchRules::member("StateSensorEvent") +
                MatchRules::path("/xyz/openbmc_project/pldm") +
                MatchRules::interface("xyz.openbmc_project.PLDM.Event"),
            std::bind(std::mem_fn(&Interface::sensorEvent), this,
                      std::placeholders::_1)),
        hostStateSignal(
            utils::getBus(),
            MatchRules::propertiesChanged("/xyz/openbmc_project/state/host0",
                                          "xyz.openbmc_project.State.Host"),
            std::bind(std::mem_fn(&Interface::hostStateEvent), this,
                      std::placeholders::_1))
    {
    }

    /** @brief Fetch the OCC state sensor PDRs and populate the cache with
     *         sensorId to OCC instance mapping information and the sensor
     *         offset for Operational Running Status.
     *
     *  @param[in] pdrs - OCC state sensor PDRs
     *  @param[out] sensorInstanceMap - map of sensorID to OCC instance
     *  @param[out] sensorOffset - sensor offset of interested state set ID
     */
    void fetchOCCSensorInfo(const PdrList& pdrs,
                            SensorToOCCInstance& sensorInstanceMap,
                            SensorOffset& sensorOffset);

    /** @brief Fetch the OCC state effecter PDRs and populate the cache with
     *         OCC instance to EffecterID information.
     *
     *  @param[in] pdrs - OCC state effecter PDRs
     *  @param[out] instanceToEffecterMap - map of OCC instance to effecterID
     *  @param[out] count - sensor offset of interested state set ID
     *  @param[out] bootRestartPos - position of Boot/Restart Cause stateSetID
     */
    void fetchOCCEffecterInfo(const PdrList& pdrs,
                              OccInstanceToEffecter& instanceToEffecterMap,
                              CompositeEffecterCount& count,
                              uint8_t& bootRestartPos);

    /** @brief Prepare the request for SetStateEffecterStates command
     *
     *  @param[in] instanceId - PLDM instanceID
     *  @param[in] instanceToEffecterMap - map of OCC instance to effecterID
     *  @param[in] count - compositeEffecterCount for OCC reset effecter PDR
     *  @param[in] bootRestartPos - position of Boot/Restart Cause stateSetID
     *
     *  @return PLDM request message to be sent to host for OCC reset, empty
     *          response in the case of failure.
     */
    std::vector<uint8_t>
        prepareSetEffecterReq(uint8_t instanceId, EffecterID effecterId,
                              CompositeEffecterCount effecterCount,
                              uint8_t bootRestartPos);

    /** @brief Send the PLDM message to reset the OCC
     *
     *  @param[in] instanceId - OCC instance to reset
     *
     */
    void resetOCC(open_power::occ::instanceID occInstanceId);

  private:
    /** @brief Callback handler to be invoked when the state of the OCC
     *         changes
     */
    std::function<bool(open_power::occ::instanceID, bool)> callBack = nullptr;

    /** @brief Used to subscribe to D-Bus PLDM StateSensorEvent signal and
     *         processes if the event corresponds to OCC state change.
     */
    sdbusplus::bus::match_t pldmEventSignal;

    /** @brief Used to subscribe for host state change signal */
    sdbusplus::bus::match_t hostStateSignal;

    /** @brief PLDM Sensor ID to OCC Instance mapping
     */
    SensorToOCCInstance sensorToOCCInstance;

    /** @brief Sensor offset of state set ID
     * PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS in state sensor PDR.
     */
    SensorOffset sensorOffset;

    /** @brief OCC Instance mapping to PLDM Effecter ID
     */
    OccInstanceToEffecter occInstanceToEffecter;

    /** @brief compositeEffecterCount for OCC reset state effecter PDR */
    CompositeEffecterCount effecterCount = 0;

    /** @brief Position of Boot/Restart Cause stateSetID in OCC state
     *         effecter PDR
     */
    uint8_t bootRestartPosition = 0;

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
};

} // namespace pldm
