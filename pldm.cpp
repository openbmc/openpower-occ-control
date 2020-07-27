#include "pldm.hpp"

#include <libpldm/entity.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>

#include <phosphor-logging/log.hpp>

namespace pldm
{

using sdbusplus::exception::SdBusError;
using namespace phosphor::logging;

void Interface::fetchOCCSensorInfo(const PdrList& pdrs,
                                   SensorToOCCInstance& sensorInstanceMap,
                                   SensorOffset& sensorOffset)
{
    bool offsetFound = false;
    auto pdr =
        reinterpret_cast<const pldm_state_sensor_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = pdr->possible_states;
    for (auto offset = 0; offset < pdr->composite_sensor_count; offset++)
    {
        auto possibleStates =
            reinterpret_cast<const state_sensor_possible_states*>(
                possibleStatesPtr);

        if (possibleStates->state_set_id ==
            PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS)
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
        log<level::ERR>("pldm: OCC state sensor PDR with StateSetId "
                        "PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS not found");
        return;
    }

    // To order SensorID based on the EntityInstance
    std::map<EntityInstance, SensorID> entityInstMap{};
    for (auto& pdr : pdrs)
    {
        auto pdrPtr =
            reinterpret_cast<const pldm_state_sensor_pdr*>(pdr.data());
        entityInstMap.emplace(
            static_cast<EntityInstance>(pdrPtr->entity_instance),
            static_cast<SensorID>(pdrPtr->sensor_id));
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
        PdrList pdrs{};

        try
        {
            auto method = bus.new_method_call(
                "xyz.openbmc_project.PLDM", "/xyz/openbmc_project/pldm",
                "xyz.openbmc_project.PLDM.PDR", "FindStateSensorPDR");
            method.append(tid, (uint16_t)PLDM_ENTITY_PROC_MODULE,
                          (uint16_t)PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS);

            auto responseMsg = bus.call(method);
            responseMsg.read(pdrs);
        }
        catch (const SdBusError& e)
        {
            log<level::ERR>("pldm: Failed to fetch the OCC state sensor PDRs",
                            entry("ERROR=%s", e.what()));
        }

        if (!pdrs.size())
        {
            log<level::ERR>("pldm: OCC state sensor PDRs not present");
            return;
        }

        fetchOCCSensorInfo(pdrs, sensorToOCCInstance, sensorOffset);
    }

    TerminusID tid{};
    SensorID sensorId{};
    SensorOffset msgSensorOffset{};
    EventState eventState{};
    EventState previousEventState{};

    msg.read(tid, sensorId, sensorOffset, eventState, previousEventState);

    auto sensorEntry = sensorToOCCInstance.find(sensorId);
    if (sensorEntry == sensorToOCCInstance.end() ||
        (msgSensorOffset != sensorOffset))
    {
        // No action for non matching sensorEvents
        return;
    }

    bool newState{};
    if (eventState == static_cast<EventState>(
                          PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
    {
        newState = callBack(sensorEntry->second, true);
    }
    else if (eventState ==
             static_cast<EventState>(
                 PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
    {
        newState = callBack(sensorEntry->second, false);
    }
    else
    {
        return;
    }

    log<level::INFO>("pldm: Updated OCCActive state",
                     entry("STATE=%s", newState ? "true" : "false"));
    return;
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
        }
    }
}

} // namespace pldm