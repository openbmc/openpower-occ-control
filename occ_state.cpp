#include "occ_state.hpp"

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
void StateChange::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    // Get the number of bytes to read
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        std::string m = "StateChange::analyzeEvent: OCC State Change Detected (failing ioctl: r=" + std::to_string(r) + ")";
        log<level::INFO>(m.c_str());
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_DEVICE_PATH(file.c_str()));
    }

    //if (manager.getNumOCCs() != occsPresent)
    //PassThrough pt = manager.passThroughObjects.front();

    auto data = readFile(len);
    if (data.empty())
    {
        log<level::INFO>("StateChange::analyzeEvent: OCC State Change Detected (empty data)");
        return;
    }
    else
    {
        log<level::INFO>("StateChange::analyzeEvent: OCC State Change Detected (normal exit)");
    }

#if 0
    if (callBack)
    {
        callBack(true);
    }
#endif
}

} // namespace occ
} // namespace open_power
