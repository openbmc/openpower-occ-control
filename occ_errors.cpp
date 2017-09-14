#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <org/open_power/OCC/Device/error.hpp>
#include "occ_errors.hpp"
#include "elog-errors.hpp"
namespace open_power
{
namespace occ
{

// Value in error file indicating success
constexpr auto NO_ERROR = '0';

using namespace phosphor::logging;
using namespace sdbusplus::org::open_power::OCC::Device::Error;
using InternalFailure = sdbusplus::xyz::openbmc_project::Common::
                                Error::InternalFailure;

// Populate the file descriptor on the error file
void Error::openFile()
{
    using namespace phosphor::logging;

    fd = open(file.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::Device::
                OpenFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::
                OpenFailure::CALLOUT_DEVICE_PATH(file.c_str()));
    }
}

// Attaches the FD to event loop and registers the callback handler
void Error::registerCallBack()
{
    decltype(eventSource.get()) sourcePtr = nullptr;
    auto r = sd_event_add_io(event.get(), &sourcePtr, fd,
                             EPOLLPRI | EPOLLERR, processEvents, this);
    eventSource.reset(sourcePtr);

    if (r < 0)
    {
        log<level::ERR>("Failed to register callback handler",
                entry("ERROR=%s", strerror(-r)));
        elog<InternalFailure>();
    }
}

// Starts to watch for errors
void Error::addWatch()
{
    if (!watching)
    {
        // Open the file
        openFile();

        // register the callback handler
        registerCallBack();

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
int Error::processEvents(sd_event_source* es, int fd,
                         uint32_t revents, void* userData)
{
    auto error = static_cast<Error*>(userData);

    error->analyzeEvent();
    return 0;
}

// Reads the error file and analyzes the data
void Error::analyzeEvent()
{
    // Get the number of bytes to read
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::
                ConfigFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::
                ConfigFailure::CALLOUT_DEVICE_PATH(file.c_str()));
    }

    // A non-zero data indicates an error condition
    // Let the caller take appropriate action on this
    auto data = readFile(len);
    bool error = !(data.empty() || data.front() == NO_ERROR);
    if (callBack)
    {
        callBack(error);
    }
    return;
}

// Reads so many bytes as passed in
std::string Error::readFile(int len) const
{
    auto data = std::make_unique<char[]>(len+1);

    // This file get created soon after binding. A value of 0 is
    // deemed success and anything else is a Failure
    // Since all the sysfs files would have size of 4096, if we read 0
    // bytes -or- value '0', then it just means we are fine
    auto r = read(fd, data.get(), len);
    if (r < 0)
    {
        elog<ReadFailure>(
            phosphor::logging::org::open_power::OCC::Device::
                ReadFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::
                ReadFailure::CALLOUT_DEVICE_PATH(file.c_str()));
    }

    // Need to seek to START, else the poll returns immediately telling
    // there is data to be read
    r = lseek(fd, 0, SEEK_SET);
    if (r < 0)
    {
        log<level::ERR>("Failure seeking error file to START");
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::
                ConfigFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::
                ConfigFailure::CALLOUT_DEVICE_PATH(file.c_str()));
    }
    return std::string(data.get());
}

} // namespace occ
} // namespace open_power
