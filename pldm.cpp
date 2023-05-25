#include "pldm.hpp"

#include "file.hpp"

#include <fmt/core.h>
#include <libpldm/entity.h>
#include "libpldm/instance-id.h"
#include <libpldm/platform.h>
#include <libpldm/state_set.h>
#include <libpldm/state_set_oem_ibm.h>
#include <libpldm/transport/mctp-demux.h>
#include <libpldm/transport.h>
#include <poll.h>

#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <algorithm>

namespace pldm
{

using namespace phosphor::logging;

using namespace sdeventplus;
using namespace sdeventplus::source;
constexpr auto clockId = sdeventplus::ClockId::RealTime;
using Clock = sdeventplus::Clock<clockId>;
using Timer = Time<clockId>;

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
            log<level::ERR>(
                fmt::format(
                    "fetchSensorInfo: Failed to find stateSetID:{} PDR: {}",
                    stateSetId, e.what())
                    .c_str());
            tracedError = true;
        }
    }

    if (pdrs.empty())
    {
        if (!tracedError)
        {
            log<level::ERR>(
                fmt::format(
                    "fetchSensorInfo: state sensor PDRs ({}) not present",
                    stateSetId)
                    .c_str());
            tracedError = true;
        }
        return;
    }

    // Found PDR
    if (tracedError)
    {
        log<level::INFO>(
            fmt::format("fetchSensorInfo: found {} PDRs", pdrs.size()).c_str());
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
        log<level::ERR>("pldm: state sensor PDR not found");
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
            bool isRunning = false;
            if (eventState ==
                static_cast<EventState>(
                    PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
            {
                log<level::INFO>(
                    fmt::format("PLDM: OCC{} is RUNNING", instance).c_str());
                isRunning = true;
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
            {
                log<level::INFO>(
                    fmt::format("PLDM: OCC{} has now STOPPED", instance)
                        .c_str());
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT))
            {
                log<level::INFO>(
                    fmt::format(
                        "PLDM: OCC{} has now STOPPED and system is in SAFE MODE",
                        instance)
                        .c_str());

                // Setting safe mode true
                safeModeCallBack(true);
            }
            else
            {
                log<level::INFO>(
                    fmt::format("PLDM: Unexpected PLDM state {} for OCC{}",
                                eventState, instance)
                        .c_str());
            }

            callBack(instance, isRunning);

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
                outstandingHResets.erase(match);
                if (eventState == static_cast<EventState>(SBE_HRESET_NOT_READY))
                {
                    log<level::INFO>(
                        fmt::format("pldm: HRESET is NOT READY (OCC{})",
                                    instance)
                            .c_str());
                }
                else if (eventState ==
                         static_cast<EventState>(SBE_HRESET_READY))
                {
                    sbeCallBack(instance, true);
                }
                else if (eventState ==
                         static_cast<EventState>(SBE_HRESET_FAILED))
                {
                    sbeCallBack(instance, false);
                }
            }
            // else request was not from us
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
        log<level::INFO>(
            fmt::format("clearData: Clearing sensorToOCCInstance ({} entries)",
                        sensorToOCCInstance.size())
                .c_str());
        for (auto entry : sensorToOCCInstance)
        {
            log<level::INFO>(
                fmt::format("clearData: OCC{} / sensorID: 0x{:04X}",
                            entry.second, entry.first)
                    .c_str());
            callBack(entry.second, false);
        }
        sensorToOCCInstance.clear();
    }
    if (!occInstanceToEffecter.empty())
    {
        log<level::DEBUG>(
            fmt::format(
                "clearData: Clearing occInstanceToEffecter ({} entries)",
                occInstanceToEffecter.size())
                .c_str());
        occInstanceToEffecter.clear();
    }
    if (!sensorToSBEInstance.empty())
    {
        log<level::DEBUG>(
            fmt::format("clearData: Clearing sensorToSBEInstance ({} entries)",
                        sensorToSBEInstance.size())
                .c_str());
        sensorToSBEInstance.clear();
    }
    if (!sbeInstanceToEffecter.empty())
    {
        log<level::DEBUG>(
            fmt::format(
                "clearData: Clearing sbeInstanceToEffecter ({} entries)",
                sbeInstanceToEffecter.size())
                .c_str());
        sbeInstanceToEffecter.clear();
    }
}

void Interface::fetchEffecterInfo(uint16_t stateSetId,
                                  InstanceToEffecter& instanceToEffecterMap,
                                  CompositeEffecterCount& effecterCount,
                                  uint8_t& stateIdPos)
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
        log<level::ERR>("pldm: Failed to fetch the state effecter PDRs",
                        entry("ERROR=%s", e.what()));
    }

    if (!pdrs.size())
    {
        log<level::ERR>("pldm: state effecter PDRs not present");
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

std::vector<uint8_t>
    Interface::prepareSetEffecterReq(EffecterID effecterId,
                                     CompositeEffecterCount effecterCount,
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
        log<level::ERR>("encode set effecter states request returned error ",
                        entry("RC=%d", rc));
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
            log<level::ERR>(
                fmt::format(
                    "pldm: Failed to find a matching effecter for OCC instance {}",
                    occInstanceId)
                    .c_str());

            return;
        }

        // Prepare the SetStateEffecterStates request to reset the OCC
        auto request = prepareSetEffecterReq(
            effecterEntry->second, OCCEffecterCount, bootRestartPosition,
            PLDM_STATE_SET_BOOT_RESTART_CAUSE_WARM_RESET);

        if (request.empty())
        {
            log<level::ERR>(
                "pldm: SetStateEffecterStates OCC reset request empty");
            return;
        }

        // Send request to reset the OCCs/PM Complex (ignore response)
        sendPldm(request, occInstanceId, false);
    }
    else
    {
        log<level::ERR>(
            fmt::format("resetOCC: HOST is not running (OCC{})", occInstanceId)
                .c_str());
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
            log<level::ERR>(
                "pldm: Failed to find a matching effecter for SBE instance",
                entry("SBE=%d", sbeInstanceId));
            return;
        }

        // Prepare the SetStateEffecterStates request to HRESET the SBE
        auto request = prepareSetEffecterReq(
            effecterEntry->second, SBEEffecterCount,
            sbeMaintenanceStatePosition, SBE_RETRY_REQUIRED);

        if (request.empty())
        {
            log<level::ERR>(
                "pldm: SetStateEffecterStates HRESET request empty");
            return;
        }

        // Send request to issue HRESET of SBE (ignore response)
        sendPldm(request, sbeInstanceId, false);
        outstandingHResets.insert(sbeInstanceId);
    }
    else
    {
        log<level::ERR>(fmt::format("sendHRESET: HOST is not running (OCC{})",
                                    sbeInstanceId)
                            .c_str());
        clearData();
    }
}

bool Interface::getPldmInstanceId()
{
    uint8_t id;
    if (!pldmInstanceID)
    {
        // Request new instance ID
        int rc = pldm_instance_id_alloc(pldmInstanceIdDb, tid, &id);
        if (rc == -EAGAIN)
        {
            rc = pldm_instance_id_alloc(pldmInstanceIdDb, tid, &id);
        }

        if (rc)
        {
            log<level::ERR>(
                fmt::format("getPldmInstanceId: Failed to alloc ID for TID {}. RC{}", tid, rc)
                    .c_str());
            return false;
        }
	pldmInstanceID.emplace(id);
            log<level::ERR>(
                fmt::format("got id {} and set PldmInstanceId to {}", id, pldmInstanceID.value())
                    .c_str());
    }

    return true;
}

bool Interface::freePldmInstanceId()
{
    if (pldmInstanceID)
    {
        int rc = pldm_instance_id_free(pldmInstanceIdDb, tid, pldmInstanceID.value());
        if (rc)
        {
            log<level::ERR>(
                fmt::format("freePldmInstanceId: Failed to free ID {} for TID {}. RC{}", pldmInstanceID.value(), tid, rc)
                    .c_str());
            return false;
        }
	pldmInstanceID.reset();
        //or pldmInstanceID = std::nullopt;
    }
    if (pldmInstanceID)
            log<level::ERR>(
                fmt::format("freed" )
                    .c_str());
    else
            log<level::ERR>(
                fmt::format("failed to freed" )
                    .c_str());

    return true;
}

int Interface::setupPldm()
{
    auto openErrno = errno;
    if (pldmTransport) {
        log<level::ERR>(
            fmt::format(
                "setupPldm: init pldm_transport already setup!, errno={}/{}",
                openErrno, strerror(openErrno))
                .c_str());
        return -1;
    }
    // Create PLDM stuffs

    int rc= pldm_transport_mctp_demux_init(&mctpTransport);
    if (rc) {
        log<level::ERR>(
            fmt::format(
                "setupPldm: Failed to init MCTP demux transport, errno={}/{}",
                rc, strerror(rc))
                .c_str());
        return -1;
    }

    pldmTransport = pldm_transport_mctp_demux_core(mctpTransport);

    if (pldm_transport_mctp_demux_map_tid(mctpTransport, tid, mctpEid)) {
        log<level::ERR>(
            fmt::format(
                "setupPldm: Failed to setup tid to eid mapping, errno={}/{}",
                openErrno, strerror(openErrno))
                .c_str());
        pldmClose();
        return -1;
    }

	struct pollfd pollfd;
	if (pldm_transport_mctp_demux_init_pollfd(mctpTransport, &pollfd)) {
        log<level::ERR>(
            fmt::format(
                "setupPldm: Failed to get pollfd , errno={}/{}",
                openErrno, strerror(openErrno))
                .c_str());
        pldmClose();
        return -1;

	}
	pldmFd = pollfd.fd;
        log<level::ERR>(
            fmt::format(
                "setupPldm: pldmFd has fd={}",
                pldmFd)
                .c_str());
	return 0;
}

void Interface::sendPldm(const std::vector<uint8_t>& request,
                         const uint8_t instance, const bool rspExpected)
{
    if (!pldmInstanceID)
    {
        log<level::ERR>("sendPldm: No PLDM Instance ID found!");
        return;
    }

    auto rc =setupPldm();
    if (rc) {
        log<level::ERR>(
            fmt::format(
                "sendPldm: setupPldm failed{}",
                rc)
                .c_str());
    }

    // Send the PLDM request message to HBRT
    if (rspExpected)
    {
        log<level::ERR>(
            fmt::format(
                "sendPldm: pldmFd has fd={}",
                pldmFd)
                .c_str());
        // Register callback when response is available
        registerPldmRspCallback();

        // Send PLDM request
        log<level::INFO>(
            fmt::format(
                "sendPldm: calling pldm_transport_send_msg(OCC{}, instance:{}, {} bytes)",
                instance, pldmInstanceID.value(), request.size())
                .c_str());
        pldmResponseReceived = false;
        pldmResponseTimeout = false;
        pldmResponseOcc = instance;
        auto pldmRc =
            pldm_transport_send_msg(pldmTransport, tid, request.data(), request.size());
        auto sendErrno = errno;
        if (pldmRc != PLDM_REQUESTER_SUCCESS)
        {
            log<level::ERR>(
                fmt::format(
                    "sendPldm: pldm_transport_send_msg failed with rc={} and errno={}/{}",
                    pldmRc, sendErrno, strerror(sendErrno))
                    .c_str());
            pldmClose();
            return;
        }

        // start timer waiting for the response
        using namespace std::literals::chrono_literals;
        pldmRspTimer.restartOnce(8s);

        // Wait for response/timeout
    }
    else // not expecting the response
    {
        log<level::INFO>(
            fmt::format(
                "sendPldm: calling pldm_transport_send_msg(tid:{}, fd:{}, {} bytes) for OCC{}",
                tid, pldmFd, request.size(), instance)
                .c_str());
        auto rc = pldm_transport_send_msg(pldmTransport, tid, request.data(), request.size());
        auto sendErrno = errno;
        if (rc)
        {
            log<level::ERR>(
                fmt::format(
                    "sendPldm: pldm_transport_send_msg(mctpID:{}, fd:{}, {} bytes) failed with rc={} and errno={}/{}",
                    mctpEid, pldmFd, request.size(), rc, sendErrno,
                    strerror(sendErrno))
                    .c_str());
        }
        else
        {
            // Not waiting for response, instance ID should be freed
	    freePldmInstanceId(); //check val?
        }
        pldmClose();
    }
}

// Attaches the FD to event loop and registers the callback handler
void Interface::registerPldmRspCallback()
{
    decltype(eventSource.get()) sourcePtr = nullptr;
    auto rc = sd_event_add_io(event.get(), &sourcePtr, pldmFd, EPOLLIN,
                              pldmRspCallback, this);
    if (rc < 0)
    {
        log<level::ERR>(
            fmt::format(
                "registerPldmRspCallback: sd_event_add_io: Error({})={} : fd={}",
                rc, strerror(-rc), pldmFd)
                .c_str());
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
        log<level::WARNING>(
            fmt::format(
                "pldmRspExpired: timerCallback - timeout waiting for pldm response for OCC{}",
                pldmResponseOcc)
                .c_str());
        pldmResponseTimeout = true;
        if (pldmFd)
        {
            pldmClose();
        }
    }
    return;
};

void Interface::pldmClose()
{
    if (pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmRspTimer.setEnabled(false);
    }

    pldm_transport_mctp_demux_destroy(mctpTransport);
    pldmFd = -1;
    eventSource.reset();
}

int Interface::pldmRspCallback(sd_event_source* /*es*/, __attribute__((unused)) int fd,
                               uint32_t revents, void* userData)
{

    if (!(revents & EPOLLIN))
    {
        log<level::INFO>(
            fmt::format("pldmRspCallback - revents={:08X}", revents).c_str());
        return -1;
    }

    auto pldmIface = static_cast<Interface*>(userData);

    if (!pldmIface->pldmInstanceID)
    {
        log<level::ERR>(
            "pldmRspCallback: No outstanding PLDM Instance ID found");
        return -1;
    }

    uint8_t* responseMsg = nullptr;
    size_t responseMsgSize{};

    log<level::INFO>(
        fmt::format("pldmRspCallback: calling pldm_recv() instance:{}",
                    pldmIface->pldmInstanceID.value())
            .c_str());
    auto rc = pldm_transport_recv_msg(pldmIface->pldmTransport, tid,
                        (void**)&responseMsg, &responseMsgSize);
    int lastErrno = errno;
    if (rc)
    {
        log<level::ERR>(
            fmt::format(
                "pldmRspCallback: pldm_transport_recv_msg failed with rc={}, errno={}/{}", rc,
                lastErrno, strerror(lastErrno))
                .c_str());
        return -1;
    }

    // We got the response for the PLDM request msg that was sent
    log<level::INFO>(
        fmt::format("pldmRspCallback: pldm_recv() rsp was {} bytes",
        fmt::format("pldmRspCallback: pldm_transport_recv_msg() rsp was {} bytes",
                    responseMsgSize)
            .c_str());

    if (pldmIface->pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmIface->pldmRspTimer.setEnabled(false);
    }

    // instance ID should be freed
    pldmIface->pldmInstanceID = std::nullopt;

    // Set pointer to autodelete
    std::unique_ptr<uint8_t, decltype(std::free)*> responseMsgPtr{responseMsg,
                                                                  std::free};

    auto response = reinterpret_cast<pldm_msg*>(responseMsgPtr.get());
    if (response->payload[0] != PLDM_SUCCESS)
    {
        log<level::ERR>(
            fmt::format("pldmRspCallback: payload[0] was not success: {}",
                        response->payload[0])
                .c_str());
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
        log<level::ERR>(
            fmt::format(
                "pldmRspCallback: decode_get_state_sensor_readings failed with rc={} and compCode={}",
                msgRc, compCode)
                .c_str());
        pldmIface->pldmClose();
        return -1;
    }

    pldmIface->pldmClose();

    const uint8_t instance = pldmIface->pldmResponseOcc;
    const uint8_t occSensorState = field[0].present_state;
    pldmIface->pldmResponseReceived = true;

    if (occSensorState == PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE)
    {
        log<level::INFO>(
            fmt::format("pldmRspCallback: OCC{} is RUNNING", instance).c_str());
        pldmIface->callBack(instance, true);
    }
    else if (occSensorState ==
             PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT)
    {
        log<level::INFO>(
            fmt::format(
                "pldmRspCallback: OCC{} has now STOPPED and system is in SAFE MODE",
                instance)
                .c_str());

        // Setting safe mode true
        pldmIface->safeModeCallBack(true);

        pldmIface->callBack(instance, false);
    }
    else
    {
        log<level::INFO>(
            fmt::format(
                "pldmRspCallback: OCC{} is not running (sensor state:{})",
                instance, occSensorState)
                .c_str());
        if (occSensorState != PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED)
        {
            const size_t rspLength = responseMsgSize + sizeof(pldm_msg_hdr);
            std::vector<std::uint8_t> pldmResponse(rspLength);
            memcpy(&pldmResponse[0], reinterpret_cast<std::uint8_t*>(response),
                   rspLength);
            log<level::ERR>(
                fmt::format(
                    "pldmRspCallback: Bad State - PLDM response ({} bytes) for OCC{}:",
                    rspLength, instance)
                    .c_str());
            dump_hex(pldmResponse);
        }
        pldmIface->callBack(instance, false);
    }

    return 0;
};

std::vector<uint8_t> Interface::encodeGetStateSensorRequest(uint8_t instance,
                                                            uint16_t sensorId)
{
    if (!getPldmInstanceId())
    {
        log<level::ERR>(
            "encodeGetStateSensorRequest: failed to getPldmInstanceId");
        return std::vector<uint8_t>();
    }

    bitfield8_t sRearm = {0};
    const size_t msgSize = sizeof(pldm_msg_hdr) +
                           PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES;
    std::vector<uint8_t> request(msgSize);

    auto msg = reinterpret_cast<pldm_msg*>(request.data());
    auto msgRc = encode_get_state_sensor_readings_req(pldmInstanceID.value(),
                                                      sensorId, sRearm, 0, msg);
    if (msgRc != PLDM_SUCCESS)
    {
        log<level::ERR>(
            fmt::format(
                "encodeGetStateSensorRequest: Failed to encode sensorId:0x{:08X} for OCC{} (rc={})",
                sensorId, instance, msgRc)
                .c_str());
    }
    return request;
}

// Initiate query of the specified OCC Active Sensor
void Interface::checkActiveSensor(uint8_t instance)
{
    static bool tracedOnce = false;
    if (pldmFd > 0)
    {
        if (!tracedOnce)
        {
            log<level::ERR>(
                fmt::format(
                    "checkActiveSensor: already waiting on OCC{} (fd={})",
                    pldmResponseOcc, pldmFd)
                    .c_str());
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
        // SensorID sID = entry->first;
        log<level::INFO>(
            fmt::format("checkActiveSensor: OCC{} / sensorID: 0x{:04X}",
                        instance, entry->first)
                .c_str());

        // Encode GetStateSensorReadings PLDM message
        auto request = encodeGetStateSensorRequest(instance, entry->first);
        if (request.empty())
        {
            return;
        }

        // Send request to PLDM and setup callback for response
        sendPldm(request, instance, true);
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "checkActiveSensor: Unable to find PLDM sensor for OCC{}",
                instance)
                .c_str());
        log<level::INFO>(
            "checkActiveSensor: fetching STATE_SET_OPERATIONAL_RUNNING_STATUS");
        fetchSensorInfo(PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS,
                        sensorToOCCInstance, OCCSensorOffset);
    }
}

} // namespace pldm
