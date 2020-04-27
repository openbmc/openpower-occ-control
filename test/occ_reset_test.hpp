#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/pldm.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MockPldmSendRecv
{
  public:
    MOCK_METHOD6(pldmSendRecv,
                 uint8_t(mctp_eid_t eid, int mctp_fd,
                         const uint8_t* pldm_req_msg, size_t req_msg_len,
                         uint8_t** pldm_resp_msg, size_t* resp_msg_len));
};

class pldmInterface
{
  public:
    uint8_t pldmSendRecv(mctp_eid_t eid, int mctp_fd,
                         const uint8_t* pldm_req_msg, size_t req_msg_len,
                         uint8_t** pldm_resp_msg, size_t* resp_msg_len) = 0;
};

class pldmConnect : public pldmInterface
{
  public:
    uint8_t pldmSendRecv(mctp_eid_t eid, int mctp_fd,
                         const uint8_t* pldm_req_msg, size_t req_msg_len,
                         uint8_t** pldm_resp_msg, size_t* resp_msg_len)
    {
        return pldm_send_recv(eid, mctp_fd, pldm_req_msg, req_msg_len,
                              pldm_resp_msg, resp_msg_len);
    }
};
