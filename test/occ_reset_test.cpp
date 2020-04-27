#include <gtest/gtest.h>

TEST(parseStateEffecterData, testGoodResponse)
{
    MockPldmSendRecv sendRecv;
    auto repo = pldm_pdr_init();
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    uint8_t compositeEffecterCount = 1;
    uint16_t effecterID = 21;

    std::vector<uint8_t> pdr(sizeof(struct pldm_state_effecter_pdr) -
                             sizeof(uint8_t) +
                             sizeof(struct state_effecter_possible_states));

    auto record = reinterpret_cast<pldm_state_effecter_pdr*>(pdr.data());
    auto possibleStates =
        reinterpret_cast<state_effecter_possible_states*>(pdr->possible_states);
    std::vector<set_effecter_state_field> stateField(
        compositeEffecterCount * sizeof(set_effecter_state_field));

    record->hdr.type = PLDM_STATE_EFFECTER_PDR;
    record->hdr.record_handle = 1;
    record->entity_type = entityType;
    record->container_id = 0;
    record->composite_effecter_count = compositeEffecterCount;
    record->effecter_id = effecterID;
    possibleStates->state_set_id = stateSetId;

    stateField.emplace_back({PLDM_REQUEST_SET, 3});

    uint8_t pldm_requester_rc;

    pldm_pdr_add(repo, pdr.data(), pdr.size(), 0, false);

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

    EXPECT_CALL(sendRecv, pldm_send_recv(0, 1, stateEffecReqMsg.data(),
                                         stateEffecReqMsg.size(),
                                         &pdrResponseMsg, &pdrResponseMsgSize))
        .WillOnce(Return(pldm_requester_rc(0)));
}
