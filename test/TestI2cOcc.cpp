#include "i2c_occ.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#ifdef I2C_OCC
namespace i2c_occ
{

namespace fs = std::experimental::filesystem;

using namespace std::string_literals;
const auto STR_4_0050 = "4-0050"s;
const auto STR_5_0051 = "5-0051"s;
const auto STR_6_0056 = "6-0056"s;
const auto STR_7_0057 = "7-0057"s;

const auto TEST_DIR = "test-dir/"s;
const auto BASE = TEST_DIR + "sys/bus/i2c/devices/";
const auto I2C_0 = BASE + "i2c-0";
const auto I2C_1 = BASE + "i2c-1";
const auto I2C_2 = BASE + "i2c-2";
const auto I2C_0_0068 = BASE + "0-0068";
const auto I2C_4_0050 = BASE + STR_4_0050;
const auto I2C_5_0051 = BASE + STR_5_0051;
const auto I2C_6_0056 = BASE + STR_6_0056;
const auto I2C_7_0057 = BASE + STR_7_0057;
const auto NAME = "/name";
const auto OCC_MASTER_NAME = "/occ_master";
const auto P8_OCC_HWMON = "p8-occ-hwmon";

const auto OTHER_STRING = "SomeOtherString123"s;

class TestUtilGetOccHwmonDevices : public testing::Test
{
  public:
    TestUtilGetOccHwmonDevices()
    {
        // Prepare env for test case
        fs::create_directories(I2C_0);
        fs::create_directories(I2C_1);
        fs::create_directories(I2C_2);
        fs::create_directories(I2C_0_0068);
        fs::create_directories(I2C_4_0050);
        fs::create_directories(I2C_5_0051);
        fs::create_directories(I2C_6_0056);
        fs::create_directories(I2C_7_0057);

        std::ofstream ofs;

        ofs.open(I2C_0 + NAME); // i2c-0 has empty name
        ofs.close();

        ofs.open(I2C_1 + NAME);
        ofs << "some text\n"; // i2c-1/name has some text
        ofs.close();

        ofs.open(I2C_2 + NAME);
        ofs << "Aspped i2c"; // i2c-2/name is aspeed i2c
        ofs.close();

        ofs.open(I2C_0_0068 + NAME);
        ofs << "other text"; // 0-0068/name is has other text
        ofs.close();

        ofs.open(I2C_4_0050 + NAME);
        ofs << "p8-occ-hwmon\n"; // 4-0050/name is p8-occ-hwmon
        ofs.close();

        ofs.open(I2C_4_0050 + OCC_MASTER_NAME);
        ofs << "0\n"; // Make 4-0050 the slave occ
        ofs.close();

        ofs.open(I2C_5_0051 + NAME);
        ofs << "p8-occ-hwmon\n"; // 5-0051/name is p8-occ-hwmon
        ofs.close();

        ofs.open(I2C_5_0051 + OCC_MASTER_NAME);
        ofs << "0\n"; // Make 5-0051 the slave occ
        ofs.close();

        ofs.open(I2C_6_0056 + NAME);
        ofs << "p8-occ-hwmon\n"; // 6-0056/name is p8-occ-hwmon
        ofs.close();

        ofs.open(I2C_6_0056 + OCC_MASTER_NAME);
        ofs << "1\n"; // Make 6-0056 the master occ
        ofs.close();

        ofs.open(I2C_7_0057 + NAME);
        ofs << "p8-occ-hwmon\n"; // 7-0057/name is p8-occ-hwmon
        ofs.close();
    }

    ~TestUtilGetOccHwmonDevices()
    {
        // Cleanup test env
        fs::remove_all(TEST_DIR);
    }
};

TEST_F(TestUtilGetOccHwmonDevices, getDevicesOK)
{
    // With test env, it shall find all the 4 p8-occ-hwmon devices
    auto ret = getOccHwmonDevices(BASE.c_str());
    EXPECT_EQ(4u, ret.size());
    // The first one shall be master occ
    EXPECT_EQ(STR_6_0056, ret[0]);
    // The left is sorted
    EXPECT_EQ(STR_4_0050, ret[1]);
    EXPECT_EQ(STR_5_0051, ret[2]);
    EXPECT_EQ(STR_7_0057, ret[3]);
}

TEST_F(TestUtilGetOccHwmonDevices, getDevicesValidDirNoDevices)
{
    // Giving a dir without valid devices,
    // it shall return an empty vector
    auto ret = getOccHwmonDevices(TEST_DIR.c_str());
    EXPECT_TRUE(ret.empty());
}

TEST_F(TestUtilGetOccHwmonDevices, getDevicesDirNotExist)
{
    // Giving a dir that does not exist,
    // it shall return an empty vector
    auto ret = getOccHwmonDevices((TEST_DIR + "not-exist").c_str());
    EXPECT_TRUE(ret.empty());
}

TEST(TestI2cDbusNames, i2cToDbus)
{
    // It shall convert 4-0050 to 4_0050
    auto str = STR_4_0050;
    i2cToDbus(str);
    EXPECT_EQ("4_0050", str);

    // It shall not modify for other strings without '-'
    str = OTHER_STRING;
    i2cToDbus(str);
    EXPECT_EQ(OTHER_STRING, str);
}

TEST(TestI2cDbusNames, dbusToI2c)
{
    // It shall convert 4_0050 to 4-0050
    auto str = "4_0050"s;
    dbusToI2c(str);
    EXPECT_EQ(STR_4_0050, str);

    // It shall not modify for other strings without '-'
    str = OTHER_STRING;
    dbusToI2c(str);
    EXPECT_EQ(OTHER_STRING, str);
}

TEST(TestI2cDbusNames, getI2cDeviceName)
{
    auto path = "/org/open_power/control/occ_4_0050"s;
    auto name = getI2cDeviceName(path);
    EXPECT_EQ(STR_4_0050, name);

    // With invalid occ path, the code shall assert
    path = "/org/open_power/control/SomeInvalidPath"s;
    EXPECT_DEATH(getI2cDeviceName(path), "");
}

} // namespace i2c_occ

#endif // I2C_OCC
