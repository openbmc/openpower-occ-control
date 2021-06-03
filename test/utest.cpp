#include "powercap.hpp"
#include "utils.hpp"

#include <experimental/filesystem>
#include <occ_events.hpp>
#include <occ_manager.hpp>

#include <gtest/gtest.h>

using namespace open_power::occ;
using namespace open_power::occ::utils;

class VerifyOccInput : public ::testing::Test
{
  public:
    VerifyOccInput() :
        rc(sd_event_default(&event)), eventP(event), manager(eventP),
        occStatus(eventP, "/test/path/occ1", manager), pcap(occStatus)
    {
        EXPECT_GE(rc, 0);
        event = nullptr;
    }
    ~VerifyOccInput()
    {
    }

    sd_event* event;
    int rc;
    open_power::occ::EventPtr eventP;

    Manager manager;
    Status occStatus;
    powercap::PowerCap pcap;
};

TEST_F(VerifyOccInput, PcapDisabled)
{
    uint32_t occInput = pcap.getOccInput(100, false);
    EXPECT_EQ(occInput, 0);
}

TEST_F(VerifyOccInput, PcapEnabled)
{
    uint32_t occInput = pcap.getOccInput(100, true);
    EXPECT_EQ(occInput, 90);
}

TEST(VerifyPathParsing, EmptyPath)
{
    std::experimental::filesystem::path path = "";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "");
}

TEST(VerifyPathParsing, FilenamePath)
{
    std::experimental::filesystem::path path = "/test/foo.bar";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "foo.bar");
}

TEST(VerifyPathParsing, DirectoryPath)
{
    std::experimental::filesystem::path path = "/test/bar/";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "bar");
}

TEST(VerifyLabelValue, checkLabelValue)
{
    const std::string value = "D0000002";

    auto labelValue = checkLabelValue(value);
    EXPECT_NE(labelValue, std::nullopt);

    std::string reType = "D0";
    uint16_t reID = 2;
    auto& [type, instanceID] = *labelValue;
    EXPECT_EQ(type, reType);
    EXPECT_EQ(instanceID, reID);
}
