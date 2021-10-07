#include "pldm.hpp"

#include "file.hpp"

#include <fmt/core.h>
#include <libpldm/entity.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>

#include <phosphor-logging/log.hpp>

#define PLDM_OEM_IBM_SBE_MAINTENANCE_STATE 32772
#define SBE_RETRY_REQUIRED 0x2

#define PLDM_OEM_IBM_SBE_HRESET_STATE 32775
#define SBE_HRESET_NOT_READY 0x1
#define SBE_HRESET_READY 0x2
#define SBE_HRESET_FAILED 0x3

namespace pldm
{

using namespace phosphor::logging;

void Interface::fetchSensorInfo(uint16_t stateSetId,
                                SensorToInstance& sensorInstanceMap,
                                SensorOffset& sensorOffset)
{
    PdrList pdrs{};

    auto& bus = open_power::occ::utils::getBus();
    try
    {
        auto method = bus.new_method_call(
            "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
            "xyz.openbmc_project.PLDM.PDR", "FindStateSensorPDR");
        method.append(tid, (uint16_t)PLDM_ENTITY_PROC, stateSetId);

        auto responseMsg = bus.call(method);
        responseMsg.read(pdrs);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("pldm: Failed to fetch the state sensor PDRs",
                        entry("ERROR=%s", e.what()));
    }

    if (pdrs.empty())
    {
        log<level::ERR>("pldm: state sensor PDRs not present");
        return;
    }

    bool offsetFound = false;
    auto pdr =
        reinterpret_cast<const pldm_state_sensor_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = pdr->possible_states;
    for (auto offset = 0; offset < pdr->composite_sensor_count; offset++)
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

        if (sensorEntry == sensorToOCCInstance.end())
        {
            return;
        }

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
            log<level::INFO>(
                fmt::format("PLDM: OCC{} has now STOPPED", sensorEntry->second)
                    .c_str());
            callBack(sensorEntry->second, false);
        }
    }
    else if (msgSensorOffset == SBESensorOffset)
    {
        auto sensorEntry = sensorToSBEInstance.find(sensorId);

        if (sensorEntry == sensorToSBEInstance.end())
        {
            return;
        }

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

void Interface::fetchEffecterInfo(uint16_t entityId, uint16_t stateSetId,
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
        method.append(tid, entityId, stateSetId);

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
    auto pdr =
        reinterpret_cast<const pldm_state_effecter_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = pdr->possible_states;
    for (auto offset = 0; offset < pdr->composite_effecter_count; offset++)
    {
        auto possibleStates =
            reinterpret_cast<const state_effecter_possible_states*>(
                possibleStatesPtr);

        if (possibleStates->state_set_id == stateSetId)
        {
            stateIdPos = offset;
            effecterCount = pdr->composite_effecter_count;
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

    std::map<EntityInstance, EffecterID> entityInstMap{};
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
        fetchEffecterInfo(
            PLDM_ENTITY_PROC_MODULE, PLDM_STATE_SET_BOOT_RESTART_CAUSE,
            occInstanceToEffecter, OCCEffecterCount, bootRestartPosition);
    }

    // Find the matching effecter for the OCC instance
    auto effecterEntry = occInstanceToEffecter.find(occInstanceId);
    if (effecterEntry == occInstanceToEffecter.end())
    {
        log<level::ERR>(
            "pldm: Failed to find a matching effecter for OCC instance",
            entry("OCC_INSTANCE_ID=%d", occInstanceId));

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

    sendPldm(request);
}

void Interface::sendHRESET(open_power::occ::instanceID sbeInstanceId)
{
    if (sbeInstanceToEffecter.empty())
    {
        fetchEffecterInfo(PLDM_ENTITY_PROC, PLDM_OEM_IBM_SBE_MAINTENANCE_STATE,
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

    sendPldm(request);
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

void Interface::sendPldm(const std::vector<uint8_t>& request)
{
    // Connect to MCTP scoket
    int fd = pldm_open();
    if (fd == -1)
    {
        log<level::ERR>("pldm: Failed to connect to MCTP socket");
        return;
    }

    open_power::occ::FileDescriptor fileFd(fd);

    // Send the PLDM request message to HBRT
    uint8_t* response = nullptr;
    size_t responseSize{};
    auto rc = pldm_send_recv(mctpEid, fileFd(), request.data(), request.size(),
                             &response, &responseSize);
    std::unique_ptr<uint8_t, decltype(std::free)*> responsePtr{response,
                                                               std::free};
    if (rc)
    {
        log<level::ERR>("pldm: pldm_send_recv failed", entry("RC=%d", rc));
    }

    uint8_t completionCode{};
    auto responseMsg = reinterpret_cast<const pldm_msg*>(responsePtr.get());
    auto rcDecode = decode_set_state_effecter_states_resp(
        responseMsg, responseSize - sizeof(pldm_msg_hdr), &completionCode);
    if (rcDecode || completionCode)
    {
        log<level::ERR>(
            "pldm: decode_set_state_effecter_states_resp returned error",
            entry("RC=%d", rcDecode),
            entry("COMPLETION_CODE=%d", completionCode));
    }
}

} // namespace pldm
