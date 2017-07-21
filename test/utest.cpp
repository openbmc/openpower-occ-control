#include <gtest/gtest.h>
#include <occ_events.hpp>
#include "powercap.hpp"

using namespace open_power::occ;

class VerifyOccInput : public ::testing::Test
{
    public:
        VerifyOccInput() :
            bus(sdbusplus::bus::new_default()),
            rc(sd_event_default(&event)),
            eventP(event),
            occStatus(bus, eventP, "/test/path"),
            pcap(bus,occStatus)
        {
            EXPECT_GE(rc, 0);
            event = nullptr;
        }
        ~VerifyOccInput()
        {}

        sdbusplus::bus::bus bus;
        sd_event* event;
        int rc;
        open_power::occ::EventPtr eventP;

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
