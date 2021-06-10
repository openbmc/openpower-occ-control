#include <occ_dbus.hpp>

#include <gtest/gtest.h>

using namespace open_power::occ::dbus;

TEST(OccDBus, MaxValue)
{
    std::string tmpPath = "/abc/def";
    double maxValue = 100.00;
    double retMaxValue = 0;

    OccDBus::getOccDBus().setMaxValue(tmpPath, maxValue);
    retMaxValue = OccDBus::getOccDBus().getMaxValue(tmpPath);

    EXPECT_EQ(maxValue, retMaxValue);
}

TEST(OccDBus, MinValue)
{
    std::string tmpPath = "/abc/def";
    double minValue = 10.00;
    double retMinValue = 0;

    OccDBus::getOccDBus().setMinValue(tmpPath, minValue);
    retMinValue = OccDBus::getOccDBus().getMinValue(tmpPath);

    EXPECT_EQ(minValue, retMinValue);
}

TEST(OccDBus, Value)
{
    std::string tmpPath = "/abc/def";
    double value = 30.00;
    double retValue = 0;

    OccDBus::getOccDBus().setMaxValue(tmpPath, value);
    retValue = OccDBus::getOccDBus().getMaxValue(tmpPath);

    EXPECT_EQ(value, retValue);
}

TEST(OccDBus, Unit)
{
    std::string tmpPath = "/abc/def";
    const std::string unit = "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
    std::string retUnit = "";

    OccDBus::getOccDBus().setUnit(tmpPath, unit);
    retUnit = OccDBus::getOccDBus().getUnit(tmpPath);

    EXPECT_EQ(unit, retUnit);
}

TEST(OccDBus, OperationalStatus)
{
    std::string tmpPath = "/abc/def";
    bool retStatus = false;

    OccDBus::getOccDBus().setOperationalStatus(tmpPath, true);
    retStatus = OccDBus::getOccDBus().getOperationalStatus(tmpPath);

    EXPECT_EQ(true, retStatus);
}