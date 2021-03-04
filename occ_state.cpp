#include "occ_state.hpp"

#include "elog-errors.hpp"
#include "occ_manager.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fmt/core.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

    using namespace phosphor::logging;

// Reads the occs_present file and analyzes the data
void StateChange::analyzeEvent()
{
    using namespace sdbusplus::org::open_power::OCC::Device::Error;
    std::string m;

    // Get the number of bytes to read
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        log<level::ERR>(fmt::format("StateChange::analyzeEvent: OCC State Change Detected (failing ioctl: rc={})",
                                    r).c_str());
        elog<ConfigFailure>(
                            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                            CALLOUT_ERRNO(errno),
                            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                            CALLOUT_DEVICE_PATH(file.c_str()));
    }

    auto data = readFile(len);
    if (data.empty())
    {
        log<level::ERR>("StateChange::analyzeEvent: OCC State Change Detected (empty data)");
        return;
    }

    auto newState = std::stoi(data, nullptr, 0);
    log<level::INFO>(fmt::format("StateChange::analyzeEvent: Kernel detected OCC State Change ({})",
                                 newState).c_str());

    if (newState == 0x03)
    {
        // Kernel detected that the OCCs went to active state
        manager.occsWentActive();
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
