#include "pldm.hpp"

#include <libpldm/entity.h>
#include <libpldm/platform.h>
#include <libpldm/state_set.h>

#include <cassert>
#include <iostream>

namespace pldm
{

using sdbusplus::exception::SdBusError;

void Interface::sensorEvent(sdbusplus::message::message& msg)
{
    std::cerr << ">> Interface::sensorEvent"
              << "\n"
              << std::endl;
    if (!isPDRCacheValid())
    {
        fetchOCCSensorInfo();
    }

    TerminusID tid;
    SensorID sensorId;
    SensorOffset msgSensorOffset;
    EventState eventState;
    EventState previousEventState;

    msg.read(tid, sensorId, sensorOffset, eventState, previousEventState);

    std::cerr << ">> sensorId = " << sensorId << std::endl;
    std::cerr << ">> sensorOffset = " << static_cast<SensorID>(msgSensorOffset)
              << std::endl;
    std::cerr << ">> eventState = " << static_cast<SensorID>(eventState)
              << std::endl;
    std::cerr << ">> previousEventState = "
              << static_cast<SensorID>(previousEventState) << std::endl;

    auto entry = sensorToOCCInstance.find(sensorId);
    if (entry == sensorToOCCInstance.end() || (msgSensorOffset != sensorOffset))
    {
        return;
    }

    if (eventState == static_cast<EventState>(
                          PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_IN_SERVICE))
    {
        callBack(entry->second, true);
    }
    else if (eventState ==
             static_cast<EventState>(
                 PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS_STOPPED))
    {
        callBack(entry->second, false);
    }

    return;
}

void Interface::fetchOCCSensorInfo()
{
    std::cerr << ">> Interface::fetchOCCSensorInfo"
              << "\n"
              << std::endl;
    constexpr uint8_t tid = 0;
    std::vector<std::vector<uint8_t>> pdrs{};

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
        std::cerr << "Failed to fetch the OCC PDRs" << e.what() << "\n";
    }

    if (!pdrs.size())
    {
        return;
    }

    bool offsetFound = false;
    auto pdr = reinterpret_cast<pldm_state_sensor_pdr*>(pdrs.front().data());
    auto possibleStatesPtr = pdr->possible_states;
    for (auto offset = 0; offset < pdr->composite_sensor_count; offset++)
    {
        auto possibleStates =
            reinterpret_cast<state_sensor_possible_states*>(possibleStatesPtr);

        if (possibleStates->state_set_id ==
            PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS)
        {
            sensorOffset = offset;
            std::cerr << ">> offset = " << offset << std::endl;
            offsetFound = true;
            break;
        }
        possibleStatesPtr += sizeof(possibleStates->state_set_id) +
                             sizeof(possibleStates->possible_states_size) +
                             possibleStates->possible_states_size;
    }

    if (!offsetFound)
    {
        std::cerr << "!offsetFound"
                  << "\n";
        return;
    }

    // To order SensorID based on the EntityInstance
    std::map<EntityInstance, SensorID> entityInstMap{};
    for (auto& pdr : pdrs)
    {
        auto pdrPtr = reinterpret_cast<pldm_state_sensor_pdr*>(pdr.data());
        entityInstMap.emplace(
            static_cast<EntityInstance>(pdrPtr->entity_instance),
            static_cast<SensorID>(pdrPtr->sensor_id));
        std::cerr << ">> pdrPtr->entity_instance = "
                  << static_cast<EntityInstance>(pdrPtr->entity_instance)
                  << std::endl;
        std::cerr << ">> pdrPtr->sensor_id = "
                  << static_cast<SensorID>(pdrPtr->sensor_id) << std::endl;
    }

    open_power::occ::instanceID count = start;
    for (auto const& pair : entityInstMap)
    {
        sensorToOCCInstance.emplace(pair.second, count);
        count++;
    }
}

} // namespace pldm