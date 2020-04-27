#include "occ_reset_test.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::Return;

TEST(parseStateEffecterData, testGoodResponse)
{
    MockPldmSendRecv sendRecv;
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    uint8_t compositeEffecterCount = 1;
    uint16_t effecterID = 21;

    std::vector<uint8_t> pdr(sizeof(struct pldm_state_effecter_pdr) -
                             sizeof(uint8_t) +
                             sizeof(struct state_effecter_possible_states));

    auto record = reinterpret_cast<pldm_state_effecter_pdr*>(pdr.data());
    auto possibleStates = reinterpret_cast<state_effecter_possible_states*>(
        record->possible_states);
    std::vector<set_effecter_state_field> stateField(
        compositeEffecterCount * sizeof(set_effecter_state_field));

    record->hdr.type = PLDM_STATE_EFFECTER_PDR;
    record->hdr.record_handle = 1;
    record->entity_type = entityType;
    record->container_id = 0;
    record->composite_effecter_count = compositeEffecterCount;
    record->effecter_id = effecterID;
    possibleStates->state_set_id = stateSetId;

    stateField.emplace_back(set_effecter_state_field{PLDM_REQUEST_SET, 3});

    std::vector<uint8_t> stateEffecReqMsg(
        sizeof(pldm_msg_hdr) + sizeof(effecterID) +
        sizeof(compositeEffecterCount) + stateField.size());

    auto stateEffecReq = reinterpret_cast<pldm_msg*>(stateEffecReqMsg.data());

    auto rc = encode_set_state_effecter_states_req(
        0, effecterID, compositeEffecterCount, stateField.data(),
        stateEffecReq);

    EXPECT_EQ(rc, PLDM_SUCCESS);

    uint8_t* pdrResponseMsg = nullptr;

    size_t pdrResponseMsgSize{};

    uint8_t completionCode;

    EXPECT_CALL(sendRecv, pldm_send_recv(0, 1, stateEffecReqMsg.data(),
                                         stateEffecReqMsg.size(),
                                         &pdrResponseMsg, &pdrResponseMsgSize))
        .Times(1);
    auto requesterRc =
        pldm_send_recv(0, 1, stateEffecReqMsg.data(), stateEffecReqMsg.size(),
                       &pdrResponseMsg, &pdrResponseMsgSize);
    std::unique_ptr<uint8_t, decltype(std::free)*> pdrResponseMsgPtr{
        pdrResponseMsg, std::free};

    auto response = reinterpret_cast<pldm_msg*>(pdrResponseMsgPtr.get());
    auto rc = decode_set_state_effecter_states_resp(
        response, pdrResponseMsgSize - sizeof(pldm_msg_hdr), &completionCode);

    EXPECT_EQ(rc, PLDM_SUCCESS);
    EXPECT_EQ(completionCode, PLDM_SUCCESS);
}
