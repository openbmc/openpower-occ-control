#include <fcntl.h>
#include <unistd.h>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include "occ_errors.hpp"
namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using InternalFailure = sdbusplus::xyz::openbmc_project::Common::
                                Error::InternalFailure;
// Populate the file descriptor on the error file
void Error::openFile()
{
    using namespace phosphor::logging;

    fd = open(file.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        log<level::ERR>("Failed to open file",
                entry("PATH=%s", file.c_str()));
        elog<InternalFailure>();
    }
}

// Attaches the FD to event loop and registers the callback handler
void Error::registerCallBack()
{
    decltype(eventSource.get()) sourcePtr = nullptr;
    auto r = sd_event_add_io(event.get(), &sourcePtr, fd,
                             EPOLLIN, processEvents, this);
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
    // Open the file
    openFile();

    // register the callback handler
    registerCallBack();
}

// Stops watching for errors
void Error::removeWatch()
{
    // Close the file
    if (fd >= 0)
    {
        close(fd);
    }

    // Reduce the reference count. Since there is only one instances
    // of add_io, this will result empty loop
    eventSource.reset();
}

// Callback handler when there is an activity on the FD
int Error::processEvents(sd_event_source* es, int fd,
                         uint32_t revents, void* userData)
{
    log<level::INFO>("Error file updated");
    auto error = static_cast<Error*>(userData);

    // A non-zero data indicates an error condition
    // Let the caller take appropriate action on this
    if (error->readFile() && error->callBack)
    {
        error->callBack();
    }
    return 0;
}

// Reads the contents. If the data contained is `non-zero`,
// then its an error
int Error::readFile()
{
    int data{};
    auto r = read(fd, &data, sizeof(data));
    if (r < 0)
    {
        log<level::ERR>("Failed to read data from file",
                entry("ERROR=%s", strerror(-r)));
        elog<InternalFailure>();
    }
    return data;
}

} // namespace occ
} // namespace open_power
