#include "occ_ffdc.hpp"

#include "elog-errors.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

static constexpr size_t max_ffdc_size = 8192;

// Reads the FFDC file and create an error log, etc.
void FFDC::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    size_t total = 0;
    auto data = std::make_unique<unsigned char[]>(max_ffdc_size);
    while (total < max_ffdc_size)
    {
        auto r = read(fd, data.get(), max_ffdc_size - total);
        if (r < 0)
        {
            elog<ReadFailure>(
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_ERRNO(errno),
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_DEVICE_PATH(file.c_str()));
            return;
        }
        if (!r)
            break;
        total += r;
    }

    // TODO: create a PEL
}

} // namespace occ
} // namespace open_power
