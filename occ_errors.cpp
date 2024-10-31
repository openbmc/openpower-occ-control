#include "occ_errors.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/lg2.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace sdbusplus::org::open_power::OCC::Device::Error;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

// Populate the file descriptor on the error file
void Error::openFile()
{
    using namespace phosphor::logging;

    fd = open(file.c_str(), O_RDONLY | O_NONBLOCK);
    const int open_errno = errno;
    if (fd < 0)
    {
        lg2::error("Error::openFile: open of {FILE} failed (errno={ERR})",
                   "FILE", file.c_str(), "ERR", open_errno);
        elog<OpenFailure>(phosphor::logging::org::open_power::OCC::Device::
                              OpenFailure::CALLOUT_ERRNO(open_errno),
                          phosphor::logging::org::open_power::OCC::Device::
                              OpenFailure::CALLOUT_DEVICE_PATH(file.c_str()));
    }
}

// Attaches the FD to event loop and registers the callback handler
void Error::registerCallBack()
{
    decltype(eventSource.get()) sourcePtr = nullptr;
    auto r = sd_event_add_io(event.get(), &sourcePtr, fd, EPOLLPRI | EPOLLERR,
                             processEvents, this);
    eventSource.reset(sourcePtr);

    if (r < 0)
    {
        lg2::error("Failed to register callback handler: error={ERR}", "ERR",
                   strerror(-r));
        elog<InternalFailure>();
    }
}

// Starts to watch for errors
void Error::addWatch(bool poll)
{
    if (!watching)
    {
        // Open the file
        openFile();

        if (poll)
        {
            // register the callback handler
            registerCallBack();
        }

        // Set we are watching the error
        watching = true;
    }
}

// Stops watching for errors
void Error::removeWatch()
{
    if (watching)
    {
        // Close the file
        if (fd >= 0)
        {
            close(fd);
        }

        // Reduce the reference count. Since there is only one instances
        // of add_io, this will result empty loop
        eventSource.reset();

        // We are no more watching the error
        watching = false;
    }
}

// Callback handler when there is an activity on the FD
int Error::processEvents(sd_event_source* /*es*/, int /*fd*/,
                         uint32_t /*revents*/, void* userData)
{
    auto error = static_cast<Error*>(userData);

    error->analyzeEvent();
    return 0;
}

// Reads the error file and analyzes the data
void Error::analyzeEvent()
{
    // Get the number of bytes to read
    int err = 0;
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_DEVICE_PATH(file.c_str()));
    }

    // A non-zero data indicates an error condition
    // Let the caller take appropriate action on this
    auto data = readFile(len);
    if (!data.empty())
        err = std::stoi(data, nullptr, 0);
    if (callBack)
    {
        callBack(err);
    }
    return;
}

// Reads so many bytes as passed in
std::string Error::readFile(int len) const
{
    auto data = std::make_unique<char[]>(len + 1);
    auto retries = 3;
    auto delay = std::chrono::milliseconds{100};

    // OCC / FSI have intermittent issues so retry all reads
    while (true)
    {
        // This file get created soon after binding. A value of 0 is
        // deemed success and anything else is a Failure
        // Since all the sysfs files would have size of 4096, if we read 0
        // bytes -or- value '0', then it just means we are fine
        auto r = read(fd, data.get(), len);
        if (r < 0)
        {
            retries--;
            if (retries == 0)
            {
                elog<ReadFailure>(
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_ERRNO(errno),
                    phosphor::logging::org::open_power::OCC::Device::
                        ReadFailure::CALLOUT_DEVICE_PATH(file.c_str()));
                break;
            }
            std::this_thread::sleep_for(delay);
            continue;
        }
        break;
    }
    // Need to seek to START, else the poll returns immediately telling
    // there is data to be read
    auto r = lseek(fd, 0, SEEK_SET);
    if (r < 0)
    {
        lg2::error("Failure seeking error file to START");
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_DEVICE_PATH(file.c_str()));
    }
    return std::string(data.get());
}

} // namespace occ
} // namespace open_power
