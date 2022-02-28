#include "pldm.hpp"

#include "file.hpp"

#include <fmt/core.h>
#include <libpldm/entity.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>
#include <libpldm/state_set_oem_ibm.h>

#include <phosphor-logging/log.hpp>

namespace pldm
{

using namespace phosphor::logging;

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
    catch (const sdbusplus::exception::exception& e)
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
        uint32_t key = (static_cast<uint32_t>(pdrPtr->container_id) << 16) |
                       static_cast<uint32_t>(pdrPtr->entity_instance);
        entityInstMap.emplace(key, static_cast<SensorID>(pdrPtr->sensor_id));
    }

    open_power::occ::instanceID count = start;
    for (auto const& pair : entityInstMap)
    {
        sensorInstanceMap.emplace(pair.second, count);
        count++;
    }
}

void Interface::sensorEvent(sdbusplus::message::message& msg)
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

    TerminusID tid{};
    SensorID sensorId{};
    SensorOffset msgSensorOffset{};
    EventState eventState{};
    EventState previousEventState{};

    msg.read(tid, sensorId, msgSensorOffset, eventState, previousEventState);

    if (msgSensorOffset == OCCSensorOffset)
    {
        auto sensorEntry = sensorToOCCInstance.find(sensorId);

        if (sensorEntry != sensorToOCCInstance.end())
        {
            if (eventState ==
                static_cast<EventState>(
                    PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
            {
                log<level::INFO>(
                    fmt::format("PLDM: OCC{} is RUNNING", sensorEntry->second)
                        .c_str());
                callBack(sensorEntry->second, true);
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
            {
                log<level::INFO>(fmt::format("PLDM: OCC{} has now STOPPED",
                                             sensorEntry->second)
                                     .c_str());
                callBack(sensorEntry->second, false);
            }
            else if (eventState ==
                     static_cast<EventState>(
                         PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_DORMANT))
            {
                log<level::INFO>(
                    fmt::format(
                        "PLDM: OCC{} has now STOPPED and system is in SAFE MODE",
                        sensorEntry->second)
                        .c_str());
                callBack(sensorEntry->second, false);
            }
            else
            {
                log<level::INFO>(
                    fmt::format("PLDM: Unexpected PLDM state {} for OCC{}",
                                eventState, sensorEntry->second)
                        .c_str());
            }

            return;
        }
    }

    if (msgSensorOffset == SBESensorOffset)
    {
        auto sensorEntry = sensorToSBEInstance.find(sensorId);

        if (sensorEntry != sensorToSBEInstance.end())
        {
            if (eventState == static_cast<EventState>(SBE_HRESET_NOT_READY))
            {
                log<level::INFO>("pldm: HRESET is NOT READY",
                                 entry("SBE=%d", sensorEntry->second));
            }
            else if (eventState == static_cast<EventState>(SBE_HRESET_READY))
            {
                sbeCallBack(sensorEntry->second, true);
            }
            else if (eventState == static_cast<EventState>(SBE_HRESET_FAILED))
            {
                sbeCallBack(sensorEntry->second, false);
            }
        }
    }
}

void Interface::hostStateEvent(sdbusplus::message::message& msg)
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
            sensorToOCCInstance.clear();
            occInstanceToEffecter.clear();

            sensorToSBEInstance.clear();
            sbeInstanceToEffecter.clear();
        }
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
    catch (const sdbusplus::exception::exception& e)
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
        uint32_t key = (static_cast<uint32_t>(pdrPtr->container_id) << 16) |
                       static_cast<uint32_t>(pdrPtr->entity_instance);
        entityInstMap.emplace(key, static_cast<SensorID>(pdrPtr->effecter_id));
    }

    open_power::occ::instanceID position = start;
    for (auto const& pair : entityInstMap)
    {
        instanceToEffecterMap.emplace(position, pair.second);
        position++;
    }
}

std::vector<uint8_t>
    Interface::prepareSetEffecterReq(uint8_t instanceId, EffecterID effecterId,
                                     CompositeEffecterCount effecterCount,
                                     uint8_t stateIdPos, uint8_t stateSetValue)
{
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
        instanceId, effecterId, effecterCount, stateField.data(), requestMsg);
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

    uint8_t instanceId{};
    if (!getMctpInstanceId(instanceId))
    {
        return;
    }

    // Prepare the SetStateEffecterStates request to reset the OCC
    auto request = prepareSetEffecterReq(
        instanceId, effecterEntry->second, OCCEffecterCount,
        bootRestartPosition, PLDM_STATE_SET_BOOT_RESTART_CAUSE_WARM_RESET);

    if (request.empty())
    {
        log<level::ERR>("pldm: SetStateEffecterStates OCC reset request empty");
        return;
    }

    // Make asynchronous call to reset the OCCs/PM Complex
    sendPldm(request, true);
}

void Interface::sendHRESET(open_power::occ::instanceID sbeInstanceId)
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

    uint8_t instanceId{};
    if (!getMctpInstanceId(instanceId))
    {
        return;
    }

    // Prepare the SetStateEffecterStates request to HRESET the SBE
    auto request = prepareSetEffecterReq(
        instanceId, effecterEntry->second, SBEEffecterCount,
        sbeMaintenanceStatePosition, SBE_RETRY_REQUIRED);

    if (request.empty())
    {
        log<level::ERR>("pldm: SetStateEffecterStates HRESET request empty");
        return;
    }

    // Make asynchronous call to do the reset
    sendPldm(request, true);
}

bool Interface::getMctpInstanceId(uint8_t& instanceId)
{
    auto& bus = open_power::occ::utils::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.Requester", "GetInstanceId");
        method.append(mctpEid);
        auto reply = bus.call(method);
        reply.read(instanceId);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("pldm: GetInstanceId returned error",
                        entry("ERROR=%s", e.what()));
        return false;
    }

    return true;
}

void Interface::sendPldm(const std::vector<uint8_t>& request, const bool async)
{
    // Connect to MCTP scoket
    int fd = pldm_open();
    if (fd == -1)
    {
        log<level::ERR>(
            fmt::format("sendPldm: Failed to connect to MCTP socket, errno={}",
                        errno)
                .c_str());
        return;
    }

    open_power::occ::FileDescriptor fileFd(fd);

    // Send the PLDM request message to HBRT
    if (async == false)
    {
        uint8_t* response = nullptr;
        size_t responseSize{};
        auto rc = pldm_send_recv(mctpEid, fileFd(), request.data(),
                                 request.size(), &response, &responseSize);
        std::unique_ptr<uint8_t, decltype(std::free)*> responsePtr{response,
                                                                   std::free};
        if (rc)
        {
            log<level::ERR>(
                fmt::format(
                    "sendPldm: pldm_send_recv({},{},req,{},...) failed with rc={} and errno={}",
                    mctpEid, fileFd(), request.size(), rc, errno)
                    .c_str());
        }

        uint8_t completionCode{};
        auto responseMsg = reinterpret_cast<const pldm_msg*>(responsePtr.get());
        auto rcDecode = decode_set_state_effecter_states_resp(
            responseMsg, responseSize - sizeof(pldm_msg_hdr), &completionCode);
        if (rcDecode || completionCode)
        {
            log<level::ERR>(
                fmt::format(
                    "sendPldm: decode_set_state_effecter_states_resp failed with rc={} and compCode={}",
                    rcDecode, completionCode)
                    .c_str());
        }
    }
    else
    {
        log<level::INFO>(fmt::format("sendPldm: calling pldm_send({}, {})",
                                     mctpEid, fileFd())
                             .c_str());
        auto rc = pldm_send(mctpEid, fileFd(), request.data(), request.size());
        if (rc)
        {
            log<level::ERR>(
                fmt::format(
                    "sendPldm: pldm_send({},{},req,{}) failed with rc={} and errno={}",
                    mctpEid, fileFd(), request.size(), rc, errno)
                    .c_str());
        }
    }
}

// Determine if the Active Sensor is available and OCC Active sensor state
bool Interface::checkActiveSensor(uint8_t instance, bool& isActive)
{
    isActive = false;

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
        SensorID sID = entry->first;

        // Encode GetStateSensorReadings PLDM message
        const uint8_t sInstance = 0;
        bitfield8_t sRearm = {0};
        const size_t msgSize =
            sizeof(pldm_msg_hdr) + PLDM_GET_STATE_SENSOR_READINGS_REQ_BYTES;
        std::vector<uint8_t> requestMsg(msgSize);
        auto msg = reinterpret_cast<pldm_msg*>(requestMsg.data());
        static int lastMsgRc = 0;
        auto msgRc = encode_get_state_sensor_readings_req(sInstance, sID,
                                                          sRearm, 0, msg);
        if (msgRc != PLDM_SUCCESS)
        {
            if (msgRc != lastMsgRc)
            {
                log<level::ERR>(
                    fmt::format(
                        "checkActiveSensor: Failed to encode sensorId:0x{:08X} for OCC{} (rc={})",
                        sID, instance, msgRc)
                        .c_str());
                lastMsgRc = msgRc;
            }
            return false;
        }

        // Connect to MCTP scoket
        int fd = pldm_open();
        if (fd == -1)
        {
            log<level::ERR>(
                fmt::format(
                    "checkActiveSensor: Failed to connect to MCTP socket, errno={}",
                    errno)
                    .c_str());
            return false;
        }
        open_power::occ::FileDescriptor fileFd(fd);

        // Send PLDM msg to get state sensor readings
        log<level::INFO>(
            fmt::format(
                "checkActiveSensor: calling pldm_send(GetStateSensorReadings, OCC{})",
                instance)
                .c_str());
        static auto lastPldmRc = PLDM_REQUESTER_SUCCESS;
        auto pldmRc = pldm_send(mctpEid, fileFd(), requestMsg.data(), msgSize);
        if (pldmRc != PLDM_REQUESTER_SUCCESS)
        {
            if (pldmRc != lastPldmRc)
            {
                log<level::ERR>(
                    fmt::format(
                        "checkActiveSensor: pldm_send(GetStateSenorReadings) failed with rc={} and errno={}",
                        pldmRc, errno)
                        .c_str());
                lastPldmRc = pldmRc;
            }
            return false;
        }

        // Read the PLDM response
        uint8_t* rspPtr = nullptr;
        size_t rspLen = 0;
        log<level::INFO>("checkActiveSensor: calling pldm_recv()");
        pldmRc = pldm_recv(mctpEid, fileFd(), msg->hdr.instance_id, &rspPtr,
                           &rspLen);
        if (pldmRc == PLDM_REQUESTER_SUCCESS)
        {
            // Decode the response
            auto msgRspPtr = reinterpret_cast<const struct pldm_msg*>(rspPtr);
            uint8_t compCode = 0, sensorCount = 1;
            get_sensor_state_field field[6];
            rspLen -= sizeof(pldm_msg_hdr);
            msgRc = decode_get_state_sensor_readings_resp(
                msgRspPtr, rspLen, &compCode, &sensorCount, field);

            if ((msgRc != PLDM_SUCCESS) || (compCode != PLDM_SUCCESS))
            {
                if (msgRc != lastMsgRc)
                {
                    log<level::ERR>(
                        fmt::format(
                            "checkActiveSensor: decode_get_state_sensor_readings failed with rc={} and compCode={}",
                            msgRc, compCode)
                            .c_str());
                    lastMsgRc = msgRc;
                }

                free(rspPtr);
                rspPtr = nullptr;
                return false;
            }

            if (field[0].present_state ==
                PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE)
            {
                isActive = true;
                log<level::INFO>(
                    fmt::format("checkActiveSensor: OCC{} is RUNNING", instance,
                                field[0].present_state)
                        .c_str());
            }
            else
            {
                log<level::WARNING>(
                    fmt::format(
                        "checkActiveSensor: OCC{} is not running (state:{})",
                        instance, field[0].present_state)
                        .c_str());
            }

            free(rspPtr);
            rspPtr = nullptr;
            lastMsgRc = 0;
            lastPldmRc = PLDM_REQUESTER_SUCCESS;
            return true;
        }
        else
        {
            if (pldmRc != lastPldmRc)
            {
                log<level::ERR>(
                    fmt::format(
                        "checkActiveSensor: pldm_recv(GetStateSenorReadings) failed with rc={} and errno={}",
                        pldmRc, errno)
                        .c_str());
                lastPldmRc = pldmRc;
            }
        }
    }

    return false;
}

} // namespace pldm
