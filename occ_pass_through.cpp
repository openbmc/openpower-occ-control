#include <memory>
#include <algorithm>
#include <fcntl.h>
#include <errno.h>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <org/open_power/OCC/PassThrough/error.hpp>
#include "occ_pass_through.hpp"
#include "elog-errors.hpp"
namespace open_power
{
namespace occ
{
namespace pass_through
{

PassThrough::PassThrough(
    sdbusplus::bus::bus& bus,
    const char* path) :
    Iface(bus, path),
    path(path),
    fd(openDevice())
{
    // Nothing to do.
}

int PassThrough::openDevice()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::PassThrough::Error;

    // Device instance number starts from 1.
    devicePath.append(std::to_string((this->path.back() - '0') + 1));

    int fd = open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0)
    {
        // This would log and terminate since its not handled.
        elog<OpenFailure>(
            phosphor::logging::org::open_power::OCC::PassThrough::
                OpenFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::PassThrough::
                OpenFailure::CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }
    return fd;
}

std::vector<int32_t> PassThrough::send(std::vector<int32_t> command)
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::PassThrough::Error;

    std::vector<int32_t> response {};

    // Amester packs data in 4 bytes
    ssize_t size = command.size() * sizeof(int32_t);
    auto rc = write((fd)(), command.data(), size);
    if (rc < 0 || (rc != size))
    {
        // This would log and terminate since its not handled.
        elog<WriteFailure>(
            phosphor::logging::org::open_power::OCC::PassThrough::
                WriteFailure::CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::PassThrough::
                WriteFailure::CALLOUT_DEVICE_PATH(devicePath.c_str()));
    }

    // Now read the response. This would be the content of occ-sram
    while(1)
    {
        int32_t data {};
        auto len = read((fd)(), &data, sizeof(data));
        if (len > 0)
        {
            response.emplace_back(data);
        }
        else if (len < 0 && errno == EAGAIN)
        {
            // We may have data coming still.
            // This driver does not need a sleep for a retry.
            continue;
        }
        else if (len == 0)
        {
            // We have read all that we can.
            break;
        }
        else
        {
            // This would log and terminate since its not handled.
            elog<ReadFailure>(
                phosphor::logging::org::open_power::OCC::PassThrough::
                    ReadFailure::CALLOUT_ERRNO(errno),
                phosphor::logging::org::open_power::OCC::PassThrough::
                    ReadFailure::CALLOUT_DEVICE_PATH(devicePath.c_str()));
        }
    }

    return response;
}

} // namespace pass_through
} // namespace occ
} // namespace open_power
