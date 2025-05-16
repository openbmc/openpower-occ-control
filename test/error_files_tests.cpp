#ifdef POWER10                                                                  // SHELDON: FIX
#include "occ_manager.hpp"
#else
#include "occ_manager_legacy.hpp"
#endif

#include <stdlib.h>

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

constexpr auto num_error_files = 8;
constexpr auto device = "occ-hwmon.1";
constexpr auto error = "occ_error";
constexpr auto errorMem = "occ_mem_throttle";
constexpr auto errorPower = "occ_dvfs_power";
constexpr auto errorTemp = "occ_dvfs_overtemp";
constexpr auto legacyDevice = "occ-hwmon.2";
constexpr auto legacyErrorTemp = "occ_dvfs_ot";
constexpr auto noError = "0";

namespace fs = std::filesystem;
using namespace open_power::occ;

class ErrorFiles : public ::testing::Test
{
  public:
    ErrorFiles() :
        rc(sd_event_default(&event)), pEvent(event), manager(pEvent),
        status(pEvent, "/dummy1", manager
#ifdef POWER10
               ,
               powerMode
#endif
        )
    {
        EXPECT_GE(rc, 0);
        event = nullptr;
    }

    virtual void SetUp()
    {
        fs::path files[num_error_files];
        char tmpDirTemplate[64];

        strcpy(tmpDirTemplate, "/tmp/occXXXXXX");
        auto path = mkdtemp(tmpDirTemplate);
        assert(path != nullptr);

        occPath = path;
        devicePath = occPath / device;
        legacyDevicePath = occPath / legacyDevice;

        fs::create_directory(devicePath);
        fs::create_directory(legacyDevicePath);

        files[0] = devicePath / error;
        files[1] = devicePath / errorMem;
        files[2] = devicePath / errorPower;
        files[3] = devicePath / errorTemp;
        files[4] = legacyDevicePath / error;
        files[5] = legacyDevicePath / errorMem;
        files[6] = legacyDevicePath / errorPower;
        files[7] = legacyDevicePath / legacyErrorTemp;

        for (const fs::path& f : files)
        {
            auto stream = std::ofstream(f.c_str());

            if (stream)
            {
                stream << noError;
            }
        }
    }

    virtual void TearDown()
    {
        fs::remove_all(occPath);
    }

    sd_event* event;
    int rc;
    open_power::occ::EventPtr pEvent;
#ifdef POWER10
    std::unique_ptr<powermode::PowerMode> powerMode = nullptr;
#endif

    Manager manager;
    Status status;

    fs::path devicePath;
    fs::path legacyDevicePath;
    fs::path occPath;
};

TEST_F(ErrorFiles, AddDeviceErrorWatch)
{
    Device occDevice(pEvent, devicePath, manager, status
#ifdef POWER10
                     ,
                     powerMode
#endif
    );

    occDevice.addErrorWatch(false);
    occDevice.removeErrorWatch();
}

// legacy OCC devices not supported on POWER10
#ifndef POWER10
TEST_F(ErrorFiles, AddLegacyDeviceErrorWatch)
{
    Device legacyOccDevice(pEvent, legacyDevicePath, manager, status);

    legacyOccDevice.addErrorWatch(false);
    legacyOccDevice.removeErrorWatch();
}
#endif
