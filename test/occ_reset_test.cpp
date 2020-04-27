#include "occ_status.hpp"

#include <libpldm/base.h>
#include <libpldm/platform.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace open_power
{

namespace occ
{

namespace send_recv
{

class MockPldmSendRecv
{
  public:
    MOCK_METHOD6(pldmSendRecv,
                 uint8_t(mctp_eid_t eid, int mctp_fd,
                         const uint8_t* pldm_req_msg, size_t req_msg_len,
                         uint8_t** pldm_resp_msg, size_t* resp_msg_len));
};

} // namespace send_recv
} // namespace occ
} // namespace open_power

using namespace open_power::occ;
using ::testing::Return;

TEST(parseStateEffecterData, testGoodResponse)
{
    using namespace open_power::occ::send_recv;
    MockPldmSendRecv sendRecv;
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    uint8_t compositeEffecterCount = 1;
    uint16_t effecterID = 21;
    uint8_t instanceId = 1;
    uint8_t mctpEid = 0;

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

    std::vector<std::vector<uint8_t>> pdrList;
    pdrList.push_back(pdr);

    std::vector<uint8_t> stateEffecReqMsg(
        sizeof(pldm_msg_hdr) + sizeof(effecterID) +
        sizeof(compositeEffecterCount) + stateField.size());

    auto stateEffecReq = reinterpret_cast<pldm_msg*>(stateEffecReqMsg.data());

    uint8_t* pdrResponseMsg = nullptr;

    size_t pdrResponseMsgSize{};

    EXPECT_CALL(sendRecv, pldmSendRecv(0, 1, stateEffecReqMsg.data(),
                                       stateEffecReqMsg.size(), &pdrResponseMsg,
                                       &pdrResponseMsgSize));

    resetOCCPLDM<MockPldmSendRecv>(&sendRecv, pdrList, mctpEid, instanceId,
                                   stateSetId);

    uint16_t retEffecterId = 0;
    uint8_t retCompEffecterCnt = 0;
    std::vector<set_effecter_state_field> retStateField{};

    auto rc = decode_set_state_effecter_states_req(
        stateEffecReq, stateEffecReqMsg.size() - sizeof(pldm_msg_hdr),
        &retEffecterId, &retCompEffecterCnt, retStateField.data());

    EXPECT_EQ(rc, PLDM_SUCCESS);
    EXPECT_EQ(effecterID, retEffecterId);
    EXPECT_EQ(retCompEffecterCnt, compositeEffecterCount);
    EXPECT_EQ(retStateField[0].set_request, PLDM_REQUEST_SET);
    EXPECT_EQ(retStateField[0].effecter_state, 3);
}
