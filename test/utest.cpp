#include "powercap.hpp"
#include "utils.hpp"

#include <occ_events.hpp>
#include <occ_manager.hpp>

#include <filesystem>

#include <gtest/gtest.h>

using namespace open_power::occ;
using namespace open_power::occ::utils;

class VerifyOccInput : public ::testing::Test
{
  public:
    VerifyOccInput() :
        rc(sd_event_default(&event)), eventP(event), manager(eventP),
        occStatus(eventP, "/test/path/occ1", manager, powerMode),
        pcap(occStatus)
    {
        EXPECT_GE(rc, 0);
        event = nullptr;
    }
    ~VerifyOccInput() {}

    sd_event* event;
    int rc;
    open_power::occ::EventPtr eventP;

    Manager manager;
    Status occStatus;

    std::unique_ptr<powermode::PowerMode> powerMode = nullptr;

    powercap::PowerCap pcap;
};

TEST(VerifyPathParsing, EmptyPath)
{
    std::filesystem::path path = "";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "");
}

TEST(VerifyPathParsing, FilenamePath)
{
    std::filesystem::path path = "/test/foo.bar";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "foo.bar");
}

TEST(VerifyPathParsing, DirectoryPath)
{
    std::filesystem::path path = "/test/bar/";
    std::string parsed = Device::getPathBack(path);

    EXPECT_STREQ(parsed.c_str(), "bar");
}
