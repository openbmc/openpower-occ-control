#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MockPldmSendRecv : public sendRecv
{
  public:
    MOCK_METHOD(uint8_t, pldm_send_recv,
                (uint8_t, int, const uint8_t*, size_t, uint8_t**, size_t),
                (const override));
};
