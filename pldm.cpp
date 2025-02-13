#include "pldm.hpp"

#include "libpldm/instance-id.h"

#include "file.hpp"

#include <libpldm/entity.h>
#include <libpldm/oem/ibm/state_set.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>
#include <libpldm/transport.h>
#include <libpldm/transport/af-mctp.h>
#include <libpldm/transport/mctp-demux.h>
#include <poll.h>

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <algorithm>

namespace pldm
{

using namespace sdeventplus;
using namespace sdeventplus::source;
constexpr auto clockId = sdeventplus::ClockId::RealTime;
using Clock = sdeventplus::Clock<clockId>;
using Timer = Time<clockId>;
bool Interface::throttleTraces = false;
enum pldm_msg_type Interface::msgType = MSG_UNDEFINED;
open_power::occ::instanceID Interface::resetInstance = 0;

void Interface::fetchSensorInfo(uint16_t stateSetId,
                                SensorToInstance& sensorInstanceMap,
                                SensorOffset& sensorOffset)
{
    PdrList pdrs{};
    static bool tracedError = false;

    auto& bus = open_power::occ::utils::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateSensorPDR");
        method.append(tid, static_cast<uint16_t>(PLDM_ENTITY_PROC), stateSetId);

        auto responseMsg = bus.call(method);
        responseMsg.read(pdrs);
    }
    catch (const sdbusplus::exception_t& e)
    {
        if (!tracedError)
        {
            lg2::error(
                "fetchSensorInfo: Failed to find stateSetID:{ID} PDR: {ERR}",
                "ID", stateSetId, "ERR", e.what());
            tracedError = true;
        }
    }

    if (pdrs.empty())
    {
        if (!tracedError)
        {
            lg2::error("fetchSensorInfo: state sensor PDRs ({ID}) not present",
                       "ID", stateSetId);
            tracedError = true;
        }
        return;
    }

    // Found PDR
    if (tracedError)
    {
        lg2::info("fetchSensorInfo: found {NUM} PDRs", "NUM", pdrs.size());
        tracedError = false;
    }

    bool offsetFound = false;
    auto stateSensorPDR =
        reinterpret_cast<const pldm_state_sensor_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = stateSensorPDR->possible_states;
    for (auto offset = 0; offset < stateSensorPDR->composite_sensor_count;
         offset++)
    {
        auto possibleStates =
            reinterpret_cast<const state_sensor_possible_states*>(
                possibleStatesPtr);

        if (possibleStates->state_set_id == stateSetId)
        {
            sensorOffset = offset;
            offsetFound = true;
            break;
        }
        possibleStatesPtr += sizeof(possibleStates->state_set_id) +
                             sizeof(possibleStates->possible_states_size) +
                             possibleStates->possible_states_size;
    }

    if (!offsetFound)
    {
        lg2::error("pldm: state sensor PDR not found");
        return;
    }

    // To order SensorID based on the EntityInstance.
    // Note that when a proc is on a DCM, the PDRs for these sensors
    // could have the same instance IDs but different container IDs.
    std::map<uint32_t, SensorID> entityInstMap{};
    for (auto& pdr : pdrs)
    {
        auto pdrPtr =
            reinterpret_cast<const pldm_state_sensor_pdr*>(pdr.data());
        uint32_t key = pdrPtr->sensor_id;
        entityInstMap.emplace(key, static_cast<SensorID>(pdrPtr->sensor_id));
    }

    open_power::occ::instanceID count = start;
    for (const auto& pair : entityInstMap)
    {
        sensorInstanceMap.emplace(pair.second, count);
        count++;
    }
}

void Interface::sensorEvent(sdbusplus::message_t& msg)
{
    if (!isOCCSensorCacheValid())
    {
        fetchSensorInfo(PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS,
                        sensorToOCCInstance, OCCSensorOffset);
    }

    if (sensorToSBEInstance.empty())
    {
        fetchSensorInfo(PLDM_OEM_IBM_SBE_HRESET_STATE, sensorToSBEInstance,
                        SBESensorOffset);
    }

    TerminusID sensorTid{};
    SensorID sensorId{};
    SensorOffset msgSensorOffset{};
    EventState eventState{};
    EventState previousEventState{};

    msg.read(sensorTid, sensorId, msgSensorOffset, eventState,
             previousEventState);

    if (msgSensorOffset == OCCSensorOffset)
    {
        auto sensorEntry = sensorToOCCInstance.find(sensorId);

        if (sensorEntry != sensorToOCCInstance.end())
        {
            const uint8_t instance = sensorEntry->second;
            bool validEvent = true;
            bool isRunning = false;
            if (eventState ==
                static_cast<EventState>(
                    PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
            {
                lg2::info("PLDM: OCC{INST} is RUNNING", "INST", instance);
                isRunning = true;
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
            {
                lg2::info("PLDM: OCC{INST} has now STOPPED", "INST", instance);
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT))
            {
                lg2::error(
                    "PLDM: OCC{INST} has now STOPPED and system is in SAFE MODE",
                    "INST", instance);

                // Setting safe mode true
                safeModeCallBack(true);
            }
            else
            {
                lg2::warning(
                    "PLDM: Unexpected OCC Active sensor state {STATE} for OCC{INST}",
                    "STATE", eventState, "INST", instance);
                validEvent = false;
            }
            if (validEvent)
            {
                if ((pldmFd > 0) && (instance == pldmResponseOcc))
                {
                    // Waiting for a response for this OCC, can stop waiting
                    pldmClose();
                }
                occActiveCallBack(instance, isRunning);
            }
            return;
        }
    }

    if (msgSensorOffset == SBESensorOffset)
    {
        auto sensorEntry = sensorToSBEInstance.find(sensorId);

        if (sensorEntry != sensorToSBEInstance.end())
        {
            const uint8_t instance = sensorEntry->second;
            auto match = std::find(outstandingHResets.begin(),
                                   outstandingHResets.end(), instance);
            if (match != outstandingHResets.end())
            {
                if (eventState == static_cast<EventState>(SBE_HRESET_NOT_READY))
                {
                    lg2::warning("pldm: HRESET is NOT READY (OCC{INST})",
                                 "INST", instance);
                    // Keep waiting for status from HRESET
                }
                else if (eventState ==
                         static_cast<EventState>(SBE_HRESET_READY))
                {
                    // Reset success, clear reset request
                    outstandingHResets.erase(match);
                    sbeCallBack(instance, true);
                }
                else if (eventState ==
                         static_cast<EventState>(SBE_HRESET_FAILED))
                {
                    // Reset failed, clear reset request and collect SBE dump
                    outstandingHResets.erase(match);
                    sbeCallBack(instance, false);
                }
                else
                {
                    lg2::warning(
                        "pldm: Unexpected HRESET state {STATE} (OCC{INST})",
                        "STATE", eventState, "INST", instance);
                }
            }
            else // request was not due to our HRESET request
            {
                if (eventState == static_cast<EventState>(SBE_HRESET_FAILED))
                {
                    lg2::error(
                        "pldm: Unexpected HRESET state {FAILED} (OCC{INST}) when HRESET not outstanding",
                        "INST", instance);

                    // No recovery from failed state, so ensure comm was stopped
                    occActiveCallBack(instance, false);
                }
            }
        }
    }
}

void Interface::hostStateEvent(sdbusplus::message_t& msg)
{
    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    msg.read(interface, properties);
    const auto stateEntry = properties.find("CurrentHostState");
    if (stateEntry != properties.end())
    {
        auto stateEntryValue = stateEntry->second;
        auto propVal = std::get<std::string>(stateEntryValue);
        if (propVal == "xyz.openbmc_project.State.Host.HostState.Off")
        {
            clearData();
        }
    }
}

void Interface::clearData()
{
    if (!sensorToOCCInstance.empty())
    {
        lg2::info("clearData: Clearing sensorToOCCInstance ({NUM} entries)",
                  "NUM", sensorToOCCInstance.size());
        for (auto entry : sensorToOCCInstance)
        {
            lg2::info("clearData: OCC{INST} / sensorID: {ID}", "INST",
                      entry.second, "ID", lg2::hex, entry.first);
            occActiveCallBack(entry.second, false);
        }
        sensorToOCCInstance.clear();
    }
    if (!occInstanceToEffecter.empty())
    {
        lg2::debug("clearData: Clearing occInstanceToEffecter ({NUM} entries)",
                   "NUM", occInstanceToEffecter.size());
        occInstanceToEffecter.clear();
    }
    if (!sensorToSBEInstance.empty())
    {
        lg2::debug("clearData: Clearing sensorToSBEInstance ({NUM} entries)",
                   "NUM", sensorToSBEInstance.size());
        sensorToSBEInstance.clear();
    }
    if (!sbeInstanceToEffecter.empty())
    {
        lg2::debug("clearData: Clearing sbeInstanceToEffecter ({NUM} entries)",
                   "NUM", sbeInstanceToEffecter.size());
        sbeInstanceToEffecter.clear();
    }
}

void Interface::fetchEffecterInfo(
    uint16_t stateSetId, InstanceToEffecter& instanceToEffecterMap,
    CompositeEffecterCount& effecterCount, uint8_t& stateIdPos)
{
    PdrList pdrs{};

    auto& bus = open_power::occ::utils::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateEffecterPDR");
        method.append(tid, static_cast<uint16_t>(PLDM_ENTITY_PROC), stateSetId);

        auto responseMsg = bus.call(method);
        responseMsg.read(pdrs);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("pldm: Failed to fetch the state effecter PDRs: {ERR}",
                   "ERR", e.what());
    }

    if (!pdrs.size())
    {
        lg2::error("pldm: state effecter PDRs not present");
        return;
    }

    bool offsetFound = false;
    auto stateEffecterPDR =
        reinterpret_cast<const pldm_state_effecter_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = stateEffecterPDR->possible_states;
    for (auto offset = 0; offset < stateEffecterPDR->composite_effecter_count;
         offset++)
    {
        auto possibleStates =
            reinterpret_cast<const state_effecter_possible_states*>(
                possibleStatesPtr);

        if (possibleStates->state_set_id == stateSetId)
        {
            stateIdPos = offset;
            effecterCount = stateEffecterPDR->composite_effecter_count;
            offsetFound = true;
            break;
        }
        possibleStatesPtr += sizeof(possibleStates->state_set_id) +
                             sizeof(possibleStates->possible_states_size) +
                             possibleStates->possible_states_size;
    }

    if (!offsetFound)
    {
        return;
    }

    std::map<uint32_t, EffecterID> entityInstMap{};
    for (auto& pdr : pdrs)
    {
        auto pdrPtr =
            reinterpret_cast<const pldm_state_effecter_pdr*>(pdr.data());
        uint32_t key = pdrPtr->effecter_id;
        entityInstMap.emplace(key, static_cast<SensorID>(pdrPtr->effecter_id));
    }

    open_power::occ::instanceID position = start;
    for (const auto& pair : entityInstMap)
    {
        instanceToEffecterMap.emplace(position, pair.second);
        position++;
    }
}

std::vector<uint8_t> Interface::prepareSetEffecterReq(
    EffecterID effecterId, CompositeEffecterCount effecterCount,
    uint8_t stateIdPos, uint8_t stateSetValue)
{
    if (!getPldmInstanceId())
    {
        return std::vector<uint8_t>();
    }

    std::vector<uint8_t> request(
        sizeof(pldm_msg_hdr) + sizeof(effecterId) + sizeof(effecterCount) +
        (effecterCount * sizeof(set_effecter_state_field)));
    auto requestMsg = reinterpret_cast<pldm_msg*>(request.data());
    std::vector<set_effecter_state_field> stateField;

    for (uint8_t effecterPos = 0; effecterPos < effecterCount; effecterPos++)
    {
        if (effecterPos == stateIdPos)
        {
            stateField.emplace_back(
                set_effecter_state_field{PLDM_REQUEST_SET, stateSetValue});
        }
        else
        {
            stateField.emplace_back(
                set_effecter_state_field{PLDM_NO_CHANGE, 0});
        }
    }
    auto rc = encode_set_state_effecter_states_req(
        pldmInstanceID.value(), effecterId, effecterCount, stateField.data(),
        requestMsg);
    if (rc != PLDM_SUCCESS)
    {
        lg2::error("encode set effecter states request returned error rc={RC}",
                   "RC", rc);
        request.clear();
    }
    return request;
}

void Interface::resetOCC(open_power::occ::instanceID occInstanceId)
{
    if (open_power::occ::utils::isHostRunning())
    {
        if (!isPDREffecterCacheValid())
        {
            fetchEffecterInfo(PLDM_STATE_SET_BOOT_RESTART_CAUSE,
                              occInstanceToEffecter, OCCEffecterCount,
                              bootRestartPosition);
        }

        // Find the matching effecter for the OCC instance
        auto effecterEntry = occInstanceToEffecter.find(occInstanceId);
        if (effecterEntry == occInstanceToEffecter.end())
        {
            lg2::error(
                "pldm: Failed to find a matching effecter for OCC instance {INST}",
                "INST", occInstanceId);

            return;
        }

        // Prepare the SetStateEffecterStates request to reset the OCC
        auto request = prepareSetEffecterReq(
            effecterEntry->second, OCCEffecterCount, bootRestartPosition,
            PLDM_STATE_SET_BOOT_RESTART_CAUSE_WARM_RESET);

        if (request.empty())
        {
            lg2::error("pldm: SetStateEffecterStates OCC reset request empty");
            return;
        }

        // Send request to reset the OCCs/PM Complex (and wait for response)
        msgType = MSG_OCC_RESET;
        resetInstance = occInstanceId;
        sendPldm(request, occInstanceId, true);
    }
    else
    {
        lg2::error("resetOCC: HOST is not running (OCC{INST})", "INST",
                   occInstanceId);
        clearData();
    }
}

void Interface::sendHRESET(open_power::occ::instanceID sbeInstanceId)
{
    if (open_power::occ::utils::isHostRunning())
    {
        if (sbeInstanceToEffecter.empty())
        {
            fetchEffecterInfo(PLDM_OEM_IBM_SBE_MAINTENANCE_STATE,
                              sbeInstanceToEffecter, SBEEffecterCount,
                              sbeMaintenanceStatePosition);
        }

        auto effecterEntry = sbeInstanceToEffecter.find(sbeInstanceId);
        if (effecterEntry == sbeInstanceToEffecter.end())
        {
            lg2::error(
                "pldm: Failed to find a matching effecter for SBE instance {INST}",
                "INST", sbeInstanceId);
            return;
        }

        // Prepare the SetStateEffecterStates request to HRESET the SBE
        auto request = prepareSetEffecterReq(
            effecterEntry->second, SBEEffecterCount,
            sbeMaintenanceStatePosition, SBE_RETRY_REQUIRED);

        if (request.empty())
        {
            lg2::error("pldm: SetStateEffecterStates HRESET request empty");
            return;
        }

        // Send request to issue HRESET of SBE (and wait for response)
        msgType = MSG_HRESET;
        resetInstance = sbeInstanceId;
        sendPldm(request, sbeInstanceId, true);
        outstandingHResets.insert(sbeInstanceId);
    }
    else
    {
        lg2::error("sendHRESET: HOST is not running (OCC{INST})", "INST",
                   sbeInstanceId);
        clearData();
    }
}

bool Interface::getPldmInstanceId()
{
    pldm_instance_id_t id;
    if (!pldmInstanceID)
    {
        // Request new instance ID
        int rc = pldm_instance_id_alloc(pldmInstanceIdDb, tid, &id);
        if (rc == -EAGAIN)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rc = pldm_instance_id_alloc(pldmInstanceIdDb, tid, &id);
        }

        if (rc)
        {
            lg2::error(
                "getPldmInstanceId: Failed to alloc ID for TID {TID}. RC{RC}",
                "TID", tid, "RC", rc);
            return false;
        }
        pldmInstanceID.emplace(id);
        if (!throttleTraces)
        {
            lg2::info("got id {ID} and set PldmInstanceId to {INST}", "ID", id,
                      "INST", pldmInstanceID.value());
        }
    }
    return true;
}

void Interface::freePldmInstanceId()
{
    if (pldmInstanceID)
    {
        int rc = pldm_instance_id_free(pldmInstanceIdDb, tid,
                                       pldmInstanceID.value());
        if (rc)
        {
            lg2::error(
                "freePldmInstanceId: Failed to free ID {ID} for TID {TID}. RC{RC}",
                "ID", pldmInstanceID.value(), "TID", tid, "RC", rc);
            return;
        }
        if (!throttleTraces)
        {
            lg2::info("Freed PLDM instance ID {ID}", "ID",
                      pldmInstanceID.value());
        }
        pldmInstanceID = std::nullopt;
    }
}

[[maybe_unused]] int Interface::openMctpDemuxTransport()
{
    impl.mctpDemux = nullptr;
    int rc = pldm_transport_mctp_demux_init(&impl.mctpDemux);
    if (rc)
    {
        lg2::error(
            "openMctpDemuxTransport: Failed to init MCTP demux transport, errno={ERR}/{STR}",
            "ERR", rc, "STR", strerror(rc));
        return -1;
    }

    if (pldm_transport_mctp_demux_map_tid(impl.mctpDemux, mctpEid, mctpEid))
    {
        lg2::error(
            "openMctpDemuxTransport: Failed to setup tid to eid mapping, errno={ERR}/{STR}",
            "ERR", errno, "STR", strerror(errno));
        pldmClose();
        return -1;
    }
    pldmTransport = pldm_transport_mctp_demux_core(impl.mctpDemux);

    struct pollfd pollfd;
    if (pldm_transport_mctp_demux_init_pollfd(pldmTransport, &pollfd))
    {
        lg2::error(
            "openMctpDemuxTransport: Failed to get pollfd , errno={ERR}/{STR}",
            "ERR", errno, "STR", strerror(errno));
        pldmClose();
        return -1;
    }
    pldmFd = pollfd.fd;
    if (!throttleTraces)
    {
        lg2::info("openMctpDemuxTransport: pldmFd has fd={FD}", "FD", pldmFd);
    }
    return 0;
}

[[maybe_unused]] int Interface::openAfMctpTransport()
{
    impl.afMctp = nullptr;
    int rc = pldm_transport_af_mctp_init(&impl.afMctp);
    if (rc)
    {
        lg2::error(
            "openAfMctpTransport: Failed to init af MCTP transport, errno={ERR}/{STR}",
            "ERR", rc, "STR", strerror(rc));
        return -1;
    }

    if (pldm_transport_af_mctp_map_tid(impl.afMctp, mctpEid, mctpEid))
    {
        lg2::error(
            "openAfMctpTransport: Failed to setup tid to eid mapping, errno={ERR}/{STR}",
            "ERR", errno, "STR", strerror(errno));
        pldmClose();
        return -1;
    }
    pldmTransport = pldm_transport_af_mctp_core(impl.afMctp);

    struct pollfd pollfd;
    if (pldm_transport_af_mctp_init_pollfd(pldmTransport, &pollfd))
    {
        lg2::error(
            "openAfMctpTransport: Failed to get pollfd , errno={ERR}/{STR}",
            "ERR", errno, "STR", strerror(errno));
        pldmClose();
        return -1;
    }
    pldmFd = pollfd.fd;
    if (!throttleTraces)
    {
        lg2::info("openAfMctpTransport: pldmFd has fd={FD}", "FD", pldmFd);
    }
    return 0;
}

int Interface::pldmOpen()
{
    if (pldmTransport)
    {
        lg2::error("pldmOpen: pldmTransport already setup!, errno={ERR}/{STR}",
                   "ERR", errno, "STR", strerror(errno));
        return -1;
    }
#if defined(PLDM_TRANSPORT_WITH_MCTP_DEMUX)
    return openMctpDemuxTransport();
#elif defined(PLDM_TRANSPORT_WITH_AF_MCTP)
    return openAfMctpTransport();
#else
    lg2::error("pldmOpen: Undefined pldmTransport!, errno={ERR}/{STR}", "ERR",
               errno, "STR", strerror(errno));
    return -1;
#endif

    return 0;
}

void Interface::sendPldm(const std::vector<uint8_t>& request,
                         const uint8_t instance, const bool rspExpected)
{
    if (!pldmInstanceID)
    {
        lg2::error("sendPldm: No PLDM Instance ID found!");
        return;
    }

    auto rc = pldmOpen();
    if (rc)
    {
        if (!throttleTraces)
        {
            lg2::error("sendPldm: pldmOpen failed rc={RC}", "RC", rc);
        }
        freePldmInstanceId();
        return;
    }

    pldm_tid_t pldmTID = static_cast<pldm_tid_t>(mctpEid);
    // Send the PLDM request message to HBRT
    if (rspExpected)
    {
        // Register callback when response is available
        registerPldmRspCallback();

        using namespace std::literals::chrono_literals;
        std::chrono::duration timeout = 8s;
        if ((msgType == MSG_OCC_RESET) || (msgType == MSG_HRESET))
        {
            timeout = 30s;
        }

        // Send PLDM request
        if (!throttleTraces)
        {
            lg2::info("sendPldm: calling pldm_transport_send_msg(OCC{INST}, "
                      "instance:{ID}, {LEN} bytes, timeout {TO})",
                      "INST", instance, "ID", pldmInstanceID.value(), "LEN",
                      request.size(), "TO", timeout.count());
        }
        pldmResponseReceived = false;
        pldmResponseTimeout = false;
        pldmResponseOcc = instance;
        auto pldmRc = pldm_transport_send_msg(pldmTransport, pldmTID,
                                              request.data(), request.size());
        auto sendErrno = errno;
        if (pldmRc != PLDM_REQUESTER_SUCCESS)
        {
            lg2::error(
                "sendPldm: pldm_transport_send_msg failed with rc={RC} and errno={ERR}/{STR}",
                "RC",
                static_cast<std::underlying_type_t<pldm_requester_error_codes>>(
                    pldmRc),
                "ERR", sendErrno, "STR", strerror(sendErrno));
            pldmClose();
            return;
        }

        // start timer waiting for the response
        pldmRspTimer.restartOnce(timeout);

        // Wait for response/timeout
    }
    else // not expecting the response
    {
        if (!throttleTraces)
        {
            lg2::info(
                "sendPldm: calling pldm_transport_send_msg(mctpID:{ID}, fd:{FD}, "
                "{LEN} bytes) for OCC{INST}",
                "ID", mctpEid, "FD", pldmFd, "LEN", request.size(), "INST",
                instance);
        }
        auto rc = pldm_transport_send_msg(pldmTransport, pldmTID,
                                          request.data(), request.size());
        auto sendErrno = errno;
        if (rc)
        {
            lg2::error(
                "sendPldm: pldm_transport_send_msg(mctpID:{ID}, fd:{FD}, {LEN} bytes) "
                "failed with rc={RC} and errno={ERR}/{STR}",
                "ID", mctpEid, "FD", pldmFd, "LEN", request.size(), "RC",
                static_cast<std::underlying_type_t<pldm_requester_error_codes>>(
                    rc),
                "ERR", sendErrno, "STR", strerror(sendErrno));
        }
        pldmClose();
    }
}

// Attaches the FD to event loop and registers the callback handler
void Interface::registerPldmRspCallback()
{
    decltype(eventSource.get()) sourcePtr = nullptr;
    int rc = 0;
    if ((msgType == MSG_OCC_RESET) || (msgType == MSG_HRESET))
    {
        rc = sd_event_add_io(event.get(), &sourcePtr, pldmFd, EPOLLIN,
                             pldmResetCallback, this);
    }
    else
    {
        rc = sd_event_add_io(event.get(), &sourcePtr, pldmFd, EPOLLIN,
                             pldmRspCallback, this);
    }
    if (rc < 0)
    {
        lg2::error(
            "registerPldmRspCallback: sd_event_add_io: Error({ERR})={STR} : fd={FD} (msgType={MSG})",
            "ERR", rc, "STR", strerror(-rc), "FD", pldmFd, "MSG", msgType);
    }
    else
    {
        // puts sourcePtr in the event source.
        eventSource.reset(sourcePtr);
    }
}

// Add a timer to the event loop, default 30s.
void Interface::pldmRspExpired()
{
    if (!pldmResponseReceived)
    {
        if (!throttleTraces)
        {
            lg2::warning(
                "pldmRspExpired: timerCallback - timeout waiting for pldm "
                "response to msg:{MSG} for OCC{INST}",
                "MSG", msgType, "INST", pldmResponseOcc);
        }
        pldmResponseTimeout = true;
        if (pldmFd)
        {
            pldmClose();
        }
        if (msgType == MSG_OCC_RESET)
        {
            // reset not acked, try again
            lg2::error("pldmRspExpired: retrying reset request for OCC{INST}",
                       "INST", pldmResponseOcc);
            resetOCC(pldmResponseOcc);
        }
    }
    return;
};

void Interface::pldmClose()
{
    freePldmInstanceId();
    if (pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmRspTimer.setEnabled(false);
    }

#if defined(PLDM_TRANSPORT_WITH_MCTP_DEMUX)
    pldm_transport_mctp_demux_destroy(impl.mctpDemux);
    impl.mctpDemux = NULL;
#elif defined(PLDM_TRANSPORT_WITH_AF_MCTP)
    pldm_transport_af_mctp_destroy(impl.afMctp);
    impl.afMctp = NULL;
#endif
    pldmFd = -1;
    pldmTransport = NULL;
    eventSource.reset();
}

int Interface::pldmRspCallback(sd_event_source* /*es*/,
                               __attribute__((unused)) int fd, uint32_t revents,
                               void* userData)
{
    if (!(revents & EPOLLIN))
    {
        lg2::info("pldmRspCallback - revents={NUM}", "NUM", lg2::hex, revents);
        return -1;
    }

    auto pldmIface = static_cast<Interface*>(userData);

    if (!pldmIface->pldmInstanceID)
    {
        lg2::error("pldmRspCallback: No outstanding PLDM Instance ID found");
        return -1;
    }

    uint8_t* responseMsg = nullptr;
    size_t responseMsgSize{};
    pldm_tid_t pldmTID = static_cast<pldm_tid_t>(mctpEid);

    if (!throttleTraces)
    {
        lg2::info(
            "pldmRspCallback: calling pldm_transport_recv_msg() instance:{INST}",
            "INST", pldmIface->pldmInstanceID.value());
    }
    auto rc = pldm_transport_recv_msg(pldmIface->pldmTransport, &pldmTID,
                                      (void**)&responseMsg, &responseMsgSize);
    int lastErrno = errno;
    if (rc)
    {
        if (!throttleTraces)
        {
            lg2::error(
                "pldmRspCallback: pldm_transport_recv_msg failed with rc={RC}, errno={ERR}/{STR}",
                "RC",
                static_cast<std::underlying_type_t<pldm_requester_error_codes>>(
                    rc),
                "ERR", lastErrno, "STR", strerror(lastErrno));
        }
        return -1;
    }

    // We got the response for the PLDM request msg that was sent
    if (!throttleTraces)
    {
        lg2::info(
            "pldmRspCallback: pldm_transport_recv_msg() rsp was {LEN} bytes",
            "LEN", responseMsgSize);
    }

    if (pldmIface->pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmIface->pldmRspTimer.setEnabled(false);
    }

    // instance ID will get freed on pldmClose()

    // Set pointer to autodelete
    std::unique_ptr<uint8_t, decltype(std::free)*> responseMsgPtr{
        responseMsg, std::free};

    auto response = reinterpret_cast<pldm_msg*>(responseMsgPtr.get());
    if (response->payload[0] != PLDM_SUCCESS)
    {
        lg2::error("pldmRspCallback: payload[0] was not success: {STATUS}",
                   "STATUS", response->payload[0]);
        pldmIface->pldmClose();
        return -1;
    }

    // Decode the response
    uint8_t compCode = 0, sensorCount = 1;
    get_sensor_state_field field[6];
    responseMsgSize -= sizeof(pldm_msg_hdr);
    auto msgRc = decode_get_state_sensor_readings_resp(
        response, responseMsgSize, &compCode, &sensorCount, field);
    if ((msgRc != PLDM_SUCCESS) || (compCode != PLDM_SUCCESS))
    {
        lg2::error(
            "pldmRspCallback: decode_get_state_sensor_readings failed with rc={RC} and compCode={CC}",
            "RC", msgRc, "CC", compCode);
        pldmIface->pldmClose();
        return -1;
    }

    pldmIface->pldmClose();

    const uint8_t instance = pldmIface->pldmResponseOcc;
    const uint8_t occSensorState = field[0].present_state;
    pldmIface->pldmResponseReceived = true;

    if (occSensorState == PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE)
    {
        lg2::info("pldmRspCallback: OCC{INST} is RUNNING", "INST", instance);
        pldmIface->occActiveCallBack(instance, true);
    }
    else if (occSensorState ==
             PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT)
    {
        lg2::error(
            "pldmRspCallback: OCC{INST} has now STOPPED and system is in SAFE MODE",
            "INST", instance);

        // Setting safe mode true
        pldmIface->safeModeCallBack(true);

        pldmIface->occActiveCallBack(instance, false);
    }
    else if (occSensorState ==
             PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED)
    {
        lg2::info("pldmRspCallback: OCC{INST} is not running", "INST",
                  instance);
        pldmIface->occActiveCallBack(instance, false);
    }
    else
    {
        const size_t rspLength = responseMsgSize + sizeof(pldm_msg_hdr);
        std::vector<std::uint8_t> pldmResponse(rspLength);
        memcpy(&pldmResponse[0], reinterpret_cast<std::uint8_t*>(response),
               rspLength);
        if (!throttleTraces)
        {
            lg2::warning(
                "pldmRspCallback: Unexpected State: {STATE} - PLDM response "
                "({LEN} bytes) for OCC{INST}:",
                "STATE", occSensorState, "LEN", rspLength, "INST", instance);
            dump_hex(pldmResponse);
        }
    }

    return 0;
};

int Interface::pldmResetCallback(sd_event_source* /*es*/,
                                 __attribute__((unused)) int fd,
                                 uint32_t revents, void* userData)
{
    if (!(revents & EPOLLIN))
    {
        lg2::info("pldmResetCallback - revents={NUM}", "NUM", lg2::hex,
                  revents);
        return -1;
    }

    auto pldmIface = static_cast<Interface*>(userData);

    if (!pldmIface->pldmInstanceID)
    {
        lg2::error("pldmResetCallback: No outstanding PLDM Instance ID found");
        return -1;
    }

    uint8_t* responseMsg = nullptr;
    size_t responseMsgSize{};
    pldm_tid_t pldmTID = static_cast<pldm_tid_t>(mctpEid);

    if (!throttleTraces)
    {
        lg2::info(
            "pldmResetCallback: calling pldm_transport_recv_msg() instance:{ID}",
            "ID", pldmIface->pldmInstanceID.value());
    }
    auto rc = pldm_transport_recv_msg(pldmIface->pldmTransport, &pldmTID,
                                      (void**)&responseMsg, &responseMsgSize);
    int lastErrno = errno;
    if (rc)
    {
        if (!throttleTraces)
        {
            lg2::error(
                "pldmResetCallback: pldm_transport_recv_msg failed with rc={RC}, errno={ERR}/{STR}",
                "RC",
                static_cast<std::underlying_type_t<pldm_requester_error_codes>>(
                    rc),
                "ERR", lastErrno, "STR", strerror(lastErrno));
        }
        return -1;
    }

    // We got the response for the PLDM request msg that was sent
    if (!throttleTraces)
    {
        lg2::info(
            "pldmResetCallback: pldm_transport_recv_msg() rsp was {LEN} bytes",
            "LEN", responseMsgSize);
    }

    if (pldmIface->pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmIface->pldmRspTimer.setEnabled(false);
    }

    // instance ID will get freed on pldmClose()

    // Set pointer to autodelete
    std::unique_ptr<uint8_t, decltype(std::free)*> responseMsgPtr{
        responseMsg, std::free};

    auto response = reinterpret_cast<pldm_msg*>(responseMsgPtr.get());
    if (response->payload[0] != PLDM_SUCCESS)
    {
        lg2::error(
            "pldmResetCallback: Reset FAILED ({MSG}) - payload[0] was not success: {STATUS}",
            "MSG", msgType, "STATUS", response->payload[0]);
        pldmIface->pldmClose();

        if (msgType == MSG_OCC_RESET)
        {
            // Retry reset request
            lg2::error(
                "pldmResetCallback: retrying reset request for OCC{INST}",
                "INST", resetInstance);
            pldmIface->resetOCC(resetInstance);
        }
        return -1;
    }
    else
    {
        lg2::info("pldmResetCallback: Reset has been successfully started");
    }

    pldmIface->pldmClose();

    pldmIface->pldmResponseReceived = true;

    return 0;
}

std::vector<uint8_t> Interface::encodeGetStateSensorRequest(uint8_t instance,
                                                            uint16_t sensorId)
{
    if (!getPldmInstanceId())
    {
        lg2::error("encodeGetStateSensorRequest: failed to getPldmInstanceId");
        return std::vector<uint8_t>();
    }

    bitfield8_t sRearm = {0};
    const size_t msgSize =
        sizeof(pldm_msg_hdr) + PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES;
    std::vector<uint8_t> request(msgSize);

    auto msg = reinterpret_cast<pldm_msg*>(request.data());
    auto msgRc = encode_get_state_sensor_readings_req(pldmInstanceID.value(),
                                                      sensorId, sRearm, 0, msg);
    if (msgRc != PLDM_SUCCESS)
    {
        lg2::error(
            "encodeGetStateSensorRequest: Failed to encode sensorId:{ID} for OCC{INST} (rc={RC})",
            "ID", lg2::hex, sensorId, "INST", instance, "RC", msgRc);
    }
    return request;
}

// Initiate query of the specified OCC Active Sensor
void Interface::checkActiveSensor(uint8_t instance)
{
    static bool tracedOnce = false;
    if (pldmFd > 0)
    {
        if (!throttleTraces && !tracedOnce)
        {
            lg2::warning(
                "checkActiveSensor: already waiting on OCC{INST} (fd={FD})",
                "INST", pldmResponseOcc, "FD", pldmFd);
            tracedOnce = true;
        }
        return;
    }
    tracedOnce = false;

    if (!isOCCSensorCacheValid())
    {
        fetchSensorInfo(PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS,
                        sensorToOCCInstance, OCCSensorOffset);
    }

    // look up sensor id (key) based on instance
    auto entry = std::find_if(
        sensorToOCCInstance.begin(), sensorToOCCInstance.end(),
        [instance](const auto& entry) { return instance == entry.second; });
    if (entry != sensorToOCCInstance.end())
    {
        // Query the OCC Active Sensor state for this instance
        if (!throttleTraces)
        {
            lg2::info("checkActiveSensor: OCC{INST} / sensorID: {ID}", "INST",
                      instance, "ID", lg2::hex, entry->first);
        }

        // Encode GetStateSensorReadings PLDM message
        auto request = encodeGetStateSensorRequest(instance, entry->first);
        if (request.empty())
        {
            return;
        }

        // Send request to PLDM and setup callback for response
        msgType = MSG_SENSOR_STATUS;
        sendPldm(request, instance, true);
    }
    else
    {
        if (!throttleTraces)
        {
            lg2::error(
                "checkActiveSensor: Unable to find PLDM sensor for OCC{INST}",
                "INST", instance);
            lg2::info(
                "checkActiveSensor: fetching STATE_SET_OPERATIONAL_RUNNING_STATUS");
        }
        fetchSensorInfo(PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS,
                        sensorToOCCInstance, OCCSensorOffset);
    }
}

void Interface::setTraceThrottle(const bool throttle)
{
    if (throttle != throttleTraces)
    {
        if (throttle)
        {
            lg2::warning("PLDM traces being throttled");
        }
        else
        {
            lg2::info("PLDM traces no longer being throttled");
        }
        throttleTraces = throttle;
    }
}

} // namespace pldm
