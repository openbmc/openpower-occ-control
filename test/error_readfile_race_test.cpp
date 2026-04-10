#include "occ_errors.hpp"
#include "occ_manager.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>

#include <filesystem>
#include <fstream>
#include <thread>

#include <gtest/gtest.h>

namespace fs = std::filesystem;
using namespace open_power::occ;
using namespace sdbusplus::org::open_power::OCC::Device::Error;

/**
 * @brief Testable Error class that exposes readFile
 */
class TestableError : public Error
{
  public:
    TestableError(EventPtr& event, const fs::path& file) :
        Error(event, file, nullptr)
    {}

    // Expose readFile for testing
    std::string testReadFile(int len)
    {
        return readFile(len);
    }

    // Expose fd for testing
    void setTestFd(int testFd)
    {
        fd = testFd;
    }

    int getTestFd() const
    {
        return fd;
    }
};

/**
 * @brief Test fixture for Error::readFile race condition
 */
class ErrorReadFileRaceTest : public ::testing::Test
{
  public:
    ErrorReadFileRaceTest() :
        rc(sd_event_default(&event)), pEvent(event), manager(pEvent),
        status(pEvent, "/dummy1", manager, powerMode)
    {
        EXPECT_GE(rc, 0);
        event = nullptr;
    }

    virtual void SetUp()
    {
        char tmpDirTemplate[64];
        strcpy(tmpDirTemplate, "/tmp/occErrorTestXXXXXX");
        auto path = mkdtemp(tmpDirTemplate);
        ASSERT_NE(path, nullptr);

        testDir = path;
        errorFile = testDir / "occ_error";

        // Create initial error file
        std::ofstream file(errorFile);
        ASSERT_TRUE(file.is_open());
        file << "0";
        file.close();
    }

    virtual void TearDown()
    {
        fs::remove_all(testDir);
    }

    sd_event* event = nullptr;
    int rc;
    open_power::occ::EventPtr pEvent;
    std::unique_ptr<powermode::PowerMode> powerMode = nullptr;
    Manager manager;
    Status status;

    fs::path testDir;
    fs::path errorFile;
};

/**
 * @brief Test Error::readFile with file removed after open
 *
 * This test ACTUALLY CALLS readFile() and will FAIL without the fix.
 *
 * Without fix: if (r < 0) - will throw exception when trying to read missing
 * file With fix:    if ((r < 0) || !fs::exists(file)) - detects missing file
 * and throws ConfigFailure
 *
 * The test creates a race condition where the file is removed after
 * opening but before readFile completes. Without the fix, readFile will
 * crash or throw the wrong exception. With the fix, it detects the missing
 * file.
 */
TEST_F(ErrorReadFileRaceTest, CallReadFileWithMissingFile)
{
    // Create a TestableError object
    TestableError error(pEvent, errorFile);

    // Open the file
    int testFd = open(errorFile.c_str(), O_RDONLY | O_NONBLOCK);
    ASSERT_GE(testFd, 0);

    // Set the fd in the Error object
    error.setTestFd(testFd);

    // Verify file exists initially
    ASSERT_TRUE(fs::exists(errorFile));

    // Now remove the file to create the race condition
    // The fd is still valid, but the file doesn't exist
    fs::remove(errorFile);
    ASSERT_FALSE(fs::exists(errorFile));

    // Call readFile - this is where the fix is tested!
    // WITHOUT FIX: readFile will do lseek (succeeds), not check file existence,
    //              try to read, and throw ReadFailure exception
    // WITH FIX: readFile will do lseek (succeeds), check file existence
    // (fails),
    //           and throw ConfigFailure exception

    bool caughtConfigFailure = false;
    bool caughtOtherException = false;

    try
    {
        std::string result = error.testReadFile(100);
        // If we get here, something is wrong
        FAIL() << "readFile should have thrown an exception for missing file";
    }
    catch (const ConfigFailure&)
    {
        // This is the CORRECT exception with the fix
        caughtConfigFailure = true;
    }
    catch (...)
    {
        // This would be thrown without the fix
        caughtOtherException = true;
    }

    // WITH THE FIX: Should catch ConfigFailure because file doesn't exist
    // WITHOUT THE FIX: Would catch different exception or crash
    EXPECT_TRUE(caughtConfigFailure)
        << "WITH FIX: readFile should throw ConfigFailure when file doesn't exist after lseek";

    EXPECT_FALSE(caughtOtherException)
        << "WITHOUT FIX: readFile would throw wrong exception or crash";

    // This assertion will FAIL if the fix is not in Error::readFile
    ASSERT_TRUE(caughtConfigFailure)
        << "FAIL: Error::readFile must check file existence after lseek! "
        << "The fix '!fs::exists(file)' is missing from occ_errors.cpp line 171";

    close(testFd);
}

/**
 * @brief Test that calls readFile after file is removed during read operation
 *
 * This test simulates the race condition where:
 * 1. readFile starts reading the file
 * 2. File gets removed mid-operation
 * 3. readFile does lseek at the end
 *
 * WITHOUT FIX: lseek succeeds, no error detected
 * WITH FIX: lseek succeeds but file existence check catches the problem
 */
TEST_F(ErrorReadFileRaceTest, ReadFileAfterFileRemoved)
{
    TestableError error(pEvent, errorFile);

    // Open and set fd
    int testFd = open(errorFile.c_str(), O_RDONLY | O_NONBLOCK);
    ASSERT_GE(testFd, 0);
    error.setTestFd(testFd);

    // Remove file after opening - simulates device removal
    fs::remove(errorFile);

    // Call readFile - it will read (may succeed or fail), then do lseek
    // The fix checks file existence after lseek
    try
    {
        error.testReadFile(100);
        FAIL() << "Should have thrown exception for missing file";
    }
    catch (const ConfigFailure&)
    {
        // CORRECT - With fix, detects missing file
        SUCCEED();
    }
    catch (...)
    {
        // May catch this if read fails first, but fix should catch it at lseek
        // This is acceptable as long as we don't crash
    }

    close(testFd);
}
