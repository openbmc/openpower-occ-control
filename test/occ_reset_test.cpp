#include "occ_status.hpp"

#include <libpldm/base.h>
#include <libpldm/platform.h>

#include <gtest/gtest.h>

using namespace open_power::occ;

TEST(parseStateEffecterData, testOccReset)
{
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    uint8_t compositeEffecterCount = 1;
    uint16_t effecterId = 21;
    uint8_t instanceId = 1;
    uint16_t entityInstance = 1;

    std::vector<uint8_t> pdr(sizeof(struct pldm_state_effecter_pdr) -
                             sizeof(uint8_t) +
                             sizeof(struct state_effecter_possible_states));

    auto record = reinterpret_cast<pldm_state_effecter_pdr*>(pdr.data());
    auto possibleStates = reinterpret_cast<state_effecter_possible_states*>(
        record->possible_states);

    record->hdr.type = PLDM_STATE_EFFECTER_PDR;
    record->hdr.record_handle = 1;
    record->entity_instance = entityInstance;
    record->entity_type = entityType;
    record->container_id = 0;
    record->composite_effecter_count = compositeEffecterCount;
    record->effecter_id = effecterId;
    possibleStates->state_set_id = stateSetId;
    possibleStates->possible_states_size = 1;
    bitfield8_t bf1{};
    bf1.byte = 3;
    possibleStates->states[0].byte = bf1.byte;

    auto requestMsg = getStateEffecterRequest(record, instanceId);
    auto request = reinterpret_cast<pldm_msg*>(requestMsg.data());

    uint16_t retEffecterId = 0;
    uint8_t retCompEffecterCnt = 0;

    state_field_for_state_effecter_set retStateField;

    auto rc = decode_set_state_effecter_states_req(
        request, requestMsg.size() - sizeof(pldm_msg_hdr), &retEffecterId,
        &retCompEffecterCnt, &retStateField);

    EXPECT_EQ(rc, PLDM_SUCCESS);
    EXPECT_EQ(effecterId, retEffecterId);
    EXPECT_EQ(retCompEffecterCnt, compositeEffecterCount);
    EXPECT_EQ(stateField[0].set_request, 1);
    EXPECT_EQ(stateField[0].effecter_state, possibleStates->states[0].byte);
}
