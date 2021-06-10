#include <occ_dbus.hpp>

#include <gtest/gtest.h>

using namespace open_power::occ::dbus;

TEST(OccDBusSensors, MaxValue)
{
    std::string tmpPath = "/abc/def";
    double maxValue = 100.00;
    double retMaxValue = 0;

    OccDBusSensors::getOccDBus().setMaxValue(tmpPath, maxValue);
    retMaxValue = OccDBusSensors::getOccDBus().getMaxValue(tmpPath);

    EXPECT_EQ(maxValue, retMaxValue);
    ASSERT_THROW(OccDBusSensors::getOccDBus().getMaxValue("/abcd/"),
                 std::invalid_argument);
}

TEST(OccDBusSensors, MinValue)
{
    std::string tmpPath = "/abc/def";
    double minValue = 10.00;
    double retMinValue = 0;

    OccDBusSensors::getOccDBus().setMinValue(tmpPath, minValue);
    retMinValue = OccDBusSensors::getOccDBus().getMinValue(tmpPath);

    EXPECT_EQ(minValue, retMinValue);
    ASSERT_THROW(OccDBusSensors::getOccDBus().getMinValue("/abcd/"),
                 std::invalid_argument);
}

TEST(OccDBusSensors, Value)
{
    std::string tmpPath = "/abc/def";
    double value = 30.00;
    double retValue = 0;

    OccDBusSensors::getOccDBus().setValue(tmpPath, value);
    retValue = OccDBusSensors::getOccDBus().getValue(tmpPath);

    EXPECT_EQ(value, retValue);
    ASSERT_THROW(OccDBusSensors::getOccDBus().getValue("/abcd/"),
                 std::invalid_argument);
}

TEST(OccDBusSensors, Unit)
{
    std::string tmpPath = "/abc/def";
    const std::string unit = "xyz.openbmc_project.Sensor.Value.Unit.DegreesC";
    std::string retUnit = "";

    OccDBusSensors::getOccDBus().setUnit(tmpPath, unit);
    retUnit = OccDBusSensors::getOccDBus().getUnit(tmpPath);

    EXPECT_EQ(unit, retUnit);
    ASSERT_THROW(OccDBusSensors::getOccDBus().getUnit("/abcd/"),
                 std::invalid_argument);
}

TEST(OccDBusSensors, OperationalStatus)
{
    std::string tmpPath = "/abc/def";
    bool retStatus = false;

    OccDBusSensors::getOccDBus().setOperationalStatus(tmpPath, true);
    retStatus = OccDBusSensors::getOccDBus().getOperationalStatus(tmpPath);

    EXPECT_EQ(true, retStatus);
    ASSERT_THROW(OccDBusSensors::getOccDBus().getOperationalStatus("/abcd/"),
                 std::invalid_argument);
}