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

#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdeventplus/clock.hpp>
#include <sdeventplus/exception.hpp>
#include <sdeventplus/source/io.hpp>
#include <sdeventplus/source/time.hpp>

#include <algorithm>
#include <format>

namespace pldm
{

using namespace phosphor::logging;

using namespace sdeventplus;
using namespace sdeventplus::source;
constexpr auto clockId = sdeventplus::ClockId::RealTime;
using Clock = sdeventplus::Clock<clockId>;
using Timer = Time<clockId>;
bool Interface::throttleTraces = false;

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
                std::format(
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
                std::format(
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
            std::format("fetchSensorInfo: found {} PDRs", pdrs.size()).c_str());
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
            bool validEvent = true;
            bool isRunning = false;
            if (eventState ==
                static_cast<EventState>(
                    PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
            {
                log<level::INFO>(
                    std::format("PLDM: OCC{} is RUNNING", instance).c_str());
                isRunning = true;
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
            {
                log<level::INFO>(
                    std::format("PLDM: OCC{} has now STOPPED", instance)
                        .c_str());
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT))
            {
                log<level::ERR>(
                    std::format(
                        "PLDM: OCC{} has now STOPPED and system is in SAFE MODE",
                        instance)
                        .c_str());

                // Setting safe mode true
                safeModeCallBack(true);
            }
            else
            {
                log<level::WARNING>(
                    std::format("PLDM: Unexpected PLDM state {} for OCC{}",
                                eventState, instance)
                        .c_str());
                validEvent = false;
            }
            if (validEvent)
            {
                if ((pldmFd > 0) && (instance == pldmResponseOcc))
                {
                    // Waiting for a response for this OCC, can stop waiting
                    pldmClose();
                }
                callBack(instance, isRunning);
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
                outstandingHResets.erase(match);
                if (eventState == static_cast<EventState>(SBE_HRESET_NOT_READY))
                {
                    log<level::INFO>(
                        std::format("pldm: HRESET is NOT READY (OCC{})",
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
            std::format("clearData: Clearing sensorToOCCInstance ({} entries)",
                        sensorToOCCInstance.size())
                .c_str());
        for (auto entry : sensorToOCCInstance)
        {
            log<level::INFO>(
                std::format("clearData: OCC{} / sensorID: 0x{:04X}",
                            entry.second, entry.first)
                    .c_str());
            callBack(entry.second, false);
        }
        sensorToOCCInstance.clear();
    }
    if (!occInstanceToEffecter.empty())
    {
        log<level::DEBUG>(
            std::format(
                "clearData: Clearing occInstanceToEffecter ({} entries)",
                occInstanceToEffecter.size())
                .c_str());
        occInstanceToEffecter.clear();
    }
    if (!sensorToSBEInstance.empty())
    {
        log<level::DEBUG>(
            std::format("clearData: Clearing sensorToSBEInstance ({} entries)",
                        sensorToSBEInstance.size())
                .c_str());
        sensorToSBEInstance.clear();
    }
    if (!sbeInstanceToEffecter.empty())
    {
        log<level::DEBUG>(
            std::format(
                "clearData: Clearing sbeInstanceToEffecter ({} entries)",
                sbeInstanceToEffecter.size())
                .c_str());
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
                std::format(
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
            std::format("resetOCC: HOST is not running (OCC{})", occInstanceId)
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
        log<level::ERR>(std::format("sendHRESET: HOST is not running (OCC{})",
                                    sbeInstanceId)
                            .c_str());
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
            log<level::ERR>(
                std::format(
                    "getPldmInstanceId: Failed to alloc ID for TID {}. RC{}",
                    tid, rc)
                    .c_str());
            return false;
        }
        pldmInstanceID.emplace(id);
        if (!throttleTraces)
        {
            log<level::INFO>(
                std::format("got id {} and set PldmInstanceId to {}", id,
                            pldmInstanceID.value())
                    .c_str());
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
            log<level::ERR>(
                std::format(
                    "freePldmInstanceId: Failed to free ID {} for TID {}. RC{}",
                    pldmInstanceID.value(), tid, rc)
                    .c_str());
            return;
        }
        if (!throttleTraces)
        {
            log<level::INFO>(
                std::format("Freed PLDM instance ID {}", pldmInstanceID.value())
                    .c_str());
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
        log<level::ERR>(
            std::format(
                "openMctpDemuxTransport: Failed to init MCTP demux transport, errno={}/{}",
                rc, strerror(rc))
                .c_str());
        return -1;
    }

    if (pldm_transport_mctp_demux_map_tid(impl.mctpDemux, mctpEid, mctpEid))
    {
        log<level::ERR>(
            std::format(
                "openMctpDemuxTransport: Failed to setup tid to eid mapping, errno={}/{}",
                errno, strerror(errno))
                .c_str());
        pldmClose();
        return -1;
    }
    pldmTransport = pldm_transport_mctp_demux_core(impl.mctpDemux);

    struct pollfd pollfd;
    if (pldm_transport_mctp_demux_init_pollfd(pldmTransport, &pollfd))
    {
        log<level::ERR>(
            std::format(
                "openMctpDemuxTransport: Failed to get pollfd , errno={}/{}",
                errno, strerror(errno))
                .c_str());
        pldmClose();
        return -1;
    }
    pldmFd = pollfd.fd;
    if (!throttleTraces)
    {
        log<level::INFO>(
            std::format("openMctpDemuxTransport: pldmFd has fd={}", pldmFd)
                .c_str());
    }
    return 0;
}

[[maybe_unused]] int Interface::openAfMctpTransport()
{
    impl.afMctp = nullptr;
    int rc = pldm_transport_af_mctp_init(&impl.afMctp);
    if (rc)
    {
        log<level::ERR>(
            std::format(
                "openAfMctpTransport: Failed to init af MCTP transport, errno={}/{}",
                rc, strerror(rc))
                .c_str());
        return -1;
    }

    if (pldm_transport_af_mctp_map_tid(impl.afMctp, mctpEid, mctpEid))
    {
        log<level::ERR>(
            std::format(
                "openAfMctpTransport: Failed to setup tid to eid mapping, errno={}/{}",
                errno, strerror(errno))
                .c_str());
        pldmClose();
        return -1;
    }
    pldmTransport = pldm_transport_af_mctp_core(impl.afMctp);

    struct pollfd pollfd;
    if (pldm_transport_af_mctp_init_pollfd(pldmTransport, &pollfd))
    {
        log<level::ERR>(
            std::format(
                "openAfMctpTransport: Failed to get pollfd , errno={}/{}",
                errno, strerror(errno))
                .c_str());
        pldmClose();
        return -1;
    }
    pldmFd = pollfd.fd;
    if (!throttleTraces)
    {
        log<level::INFO>(
            std::format("openAfMctpTransport: pldmFd has fd={}", pldmFd)
                .c_str());
    }
    return 0;
}

int Interface::pldmOpen()
{
    if (pldmTransport)
    {
        log<level::ERR>(
            std::format("pldmOpen: pldmTransport already setup!, errno={}/{}",
                        errno, strerror(errno))
                .c_str());
        return -1;
    }
#if defined(PLDM_TRANSPORT_WITH_MCTP_DEMUX)
    return openMctpDemuxTransport();
#elif defined(PLDM_TRANSPORT_WITH_AF_MCTP)
    return openAfMctpTransport();
#else
    return -1;
#endif

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

    auto rc = pldmOpen();
    if (rc)
    {
        if (!throttleTraces)
        {
            log<level::ERR>(
                std::format("sendPldm: pldmOpen failed rc={}", rc).c_str());
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

        // Send PLDM request
        if (!throttleTraces)
        {
            log<level::INFO>(
                std::format(
                    "sendPldm: calling pldm_transport_send_msg(OCC{}, instance:{}, {} bytes)",
                    instance, pldmInstanceID.value(), request.size())
                    .c_str());
        }
        pldmResponseReceived = false;
        pldmResponseTimeout = false;
        pldmResponseOcc = instance;
        auto pldmRc = pldm_transport_send_msg(pldmTransport, pldmTID,
                                              request.data(), request.size());
        auto sendErrno = errno;
        if (pldmRc != PLDM_REQUESTER_SUCCESS)
        {
            log<level::ERR>(
                std::format(
                    "sendPldm: pldm_transport_send_msg failed with rc={} and errno={}/{}",
                    static_cast<
                        std::underlying_type_t<pldm_requester_error_codes>>(
                        pldmRc),
                    sendErrno, strerror(sendErrno))
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
        if (!throttleTraces)
        {
            log<level::INFO>(
                std::format(
                    "sendPldm: calling pldm_transport_send_msg(mctpID:{}, fd:{}, {} bytes) for OCC{}",
                    mctpEid, pldmFd, request.size(), instance)
                    .c_str());
        }
        auto rc = pldm_transport_send_msg(pldmTransport, pldmTID,
                                          request.data(), request.size());
        auto sendErrno = errno;
        if (rc)
        {
            log<level::ERR>(
                std::format(
                    "sendPldm: pldm_transport_send_msg(mctpID:{}, fd:{}, {} bytes) failed with rc={} and errno={}/{}",
                    mctpEid, pldmFd, request.size(),
                    static_cast<
                        std::underlying_type_t<pldm_requester_error_codes>>(rc),
                    sendErrno, strerror(sendErrno))
                    .c_str());
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
            std::format(
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
        if (!throttleTraces)
        {
            log<level::WARNING>(
                std::format(
                    "pldmRspExpired: timerCallback - timeout waiting for pldm response for OCC{}",
                    pldmResponseOcc)
                    .c_str());
        }
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
        log<level::INFO>(
            std::format("pldmRspCallback - revents={:08X}", revents).c_str());
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
    pldm_tid_t pldmTID = static_cast<pldm_tid_t>(mctpEid);

    if (!throttleTraces)
    {
        log<level::INFO>(
            std::format(
                "pldmRspCallback: calling pldm_transport_recv_msg() instance:{}",
                pldmIface->pldmInstanceID.value())
                .c_str());
    }
    auto rc = pldm_transport_recv_msg(pldmIface->pldmTransport, &pldmTID,
                                      (void**)&responseMsg, &responseMsgSize);
    int lastErrno = errno;
    if (rc)
    {
        if (!throttleTraces)
        {
            log<level::ERR>(
                std::format(
                    "pldmRspCallback: pldm_transport_recv_msg failed with rc={}, errno={}/{}",
                    static_cast<
                        std::underlying_type_t<pldm_requester_error_codes>>(rc),
                    lastErrno, strerror(lastErrno))
                    .c_str());
        }
        return -1;
    }

    // We got the response for the PLDM request msg that was sent
    if (!throttleTraces)
    {
        log<level::INFO>(
            std::format(
                "pldmRspCallback: pldm_transport_recv_msg() rsp was {} bytes",
                responseMsgSize)
                .c_str());
    }

    if (pldmIface->pldmRspTimer.isEnabled())
    {
        // stop PLDM response timer
        pldmIface->pldmRspTimer.setEnabled(false);
    }

    // instance ID should be freed
    pldmIface->pldmInstanceID = std::nullopt;

    // Set pointer to autodelete
    std::unique_ptr<uint8_t, decltype(std::free)*> responseMsgPtr{
        responseMsg, std::free};

    auto response = reinterpret_cast<pldm_msg*>(responseMsgPtr.get());
    if (response->payload[0] != PLDM_SUCCESS)
    {
        log<level::ERR>(
            std::format("pldmRspCallback: payload[0] was not success: {}",
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
            std::format(
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
            std::format("pldmRspCallback: OCC{} is RUNNING", instance).c_str());
        pldmIface->callBack(instance, true);
    }
    else if (occSensorState ==
             PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT)
    {
        log<level::ERR>(
            std::format(
                "pldmRspCallback: OCC{} has now STOPPED and system is in SAFE MODE",
                instance)
                .c_str());

        // Setting safe mode true
        pldmIface->safeModeCallBack(true);

        pldmIface->callBack(instance, false);
    }
    else if (occSensorState ==
             PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED)
    {
        log<level::INFO>(
            std::format("pldmRspCallback: OCC{} is not running", instance)
                .c_str());
        pldmIface->callBack(instance, false);
    }
    else
    {
        const size_t rspLength = responseMsgSize + sizeof(pldm_msg_hdr);
        std::vector<std::uint8_t> pldmResponse(rspLength);
        memcpy(&pldmResponse[0], reinterpret_cast<std::uint8_t*>(response),
               rspLength);
        if (!throttleTraces)
        {
            log<level::WARNING>(
                std::format(
                    "pldmRspCallback: Unexpected State: {} - PLDM response ({} bytes) for OCC{}:",
                    occSensorState, rspLength, instance)
                    .c_str());
            dump_hex(pldmResponse);
        }
    }

    return 0;
};

std::vector<uint8_t>
    Interface::encodeGetStateSensorRequest(uint8_t instance, uint16_t sensorId)
{
    if (!getPldmInstanceId())
    {
        log<level::ERR>(
            "encodeGetStateSensorRequest: failed to getPldmInstanceId");
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
        log<level::ERR>(
            std::format(
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
        if (!throttleTraces && !tracedOnce)
        {
            log<level::WARNING>(
                std::format(
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
        if (!throttleTraces)
        {
            log<level::INFO>(
                std::format("checkActiveSensor: OCC{} / sensorID: 0x{:04X}",
                            instance, entry->first)
                    .c_str());
        }

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
        if (!throttleTraces)
        {
            log<level::ERR>(
                std::format(
                    "checkActiveSensor: Unable to find PLDM sensor for OCC{}",
                    instance)
                    .c_str());
            log<level::INFO>(
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
            log<level::WARNING>("PLDM traces being throttled");
        }
        else
        {
            log<level::INFO>("PLDM traces no longer being throttled");
        }
        throttleTraces = throttle;
    }
}

} // namespace pldm
