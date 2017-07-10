#include <gtest/gtest.h>
#include "powercap.hpp"

using namespace open_power::occ;

class VerifyOccInput : public ::testing::Test
{
    public:
        VerifyOccInput() :
            bus(sdbusplus::bus::new_default()),
            occStatus(bus,"/test/path"),
            pcap(bus,occStatus)
        {}
        ~VerifyOccInput()
        {}

        sdbusplus::bus::bus bus;
        Status occStatus;
        powercap::PowerCap pcap;
};

TEST_F(VerifyOccInput, PcapDisabled) {
    uint32_t occInput = pcap.getOccInput(100,false);
    EXPECT_EQ(occInput, 0);
}

TEST_F(VerifyOccInput, PcapEnabled) {
    uint32_t occInput = pcap.getOccInput(100,true);
    EXPECT_EQ(occInput, 90);
}
