#include <libpldm/base.h>
#include <libpldm/platform.h>
#include <libpldm/pldm.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MockPldmSendRecv
{
  public:
    MOCK_METHOD(pldm_send_recv, uint8_t(uint8_t, int, const uint8_t*, size_t,
                                        uint8_t**, size_t*));
};
