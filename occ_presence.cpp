#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sstream>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <org/open_power/OCC/Device/error.hpp>
#include "occ_presence.hpp"
#include "occ_manager.hpp"
#include "elog-errors.hpp"

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;
using namespace sdbusplus::org::open_power::OCC::Device::Error;

// Reads the occs_present file and analyzes the data
void Presence::analyzeEvent()
{
    std::stringstream format;
    int occs_present;
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

    auto data = readFile(len);
    if (data.empty())
        return;

    format << data;
    format >> occs_present;

    if (manager.getNumOCCs() != occs_present)
    {
        log<level::INFO>("OCC presence mismatch",
                         entry("BMC_OCCS=%d", manager.getNumOCCs(),
                         entry("OCC_OCCS=%d", occs_present)));
        if (callBack)
        {
            callBack();
        }
    }

    return;
}

} // namespace occ
} // namespace open_power
