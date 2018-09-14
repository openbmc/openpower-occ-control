#include "occ_presence.hpp"

#include "elog-errors.hpp"
#include "occ_manager.hpp"

#include <errno.h>
#include <fcntl.h>
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

// Reads the occs_present file and analyzes the data
void Presence::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    // Get the number of bytes to read
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

    auto data = readFile(len);
    if (data.empty())
    {
        return;
    }

    // Let stoi determine the base
    auto occsPresent = std::stoi(data, nullptr, 0);
    if (manager.getNumOCCs() != occsPresent)
    {
        log<level::INFO>("OCC presence mismatch",
                         entry("BMC_OCCS=%d", manager.getNumOCCs()),
                         entry("OCC_OCCS=%d", occsPresent));
        if (callBack)
        {
            callBack(true);
        }
    }
}

} // namespace occ
} // namespace open_power
