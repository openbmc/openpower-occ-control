#include "config.h"

#include "elog-errors.hpp"
#include "occ_events.hpp"
#include "occ_manager.hpp"
#include "utils.hpp"
#ifdef POWER10
#include "powermode.hpp"
#endif

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

using namespace phosphor::logging;

using namespace sdbusplus::org::open_power::OCC::Device::Error;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

int main(int /*argc*/, char** /*argv[]*/)
{
    auto& bus = open_power::occ::utils::getBus();

    // Need sd_event to watch for OCC device errors
    sd_event* event = nullptr;
    auto r = sd_event_default(&event);
    if (r < 0)
    {
        log<level::ERR>("Error creating a default sd_event handler");
        return r;
    }
    open_power::occ::EventPtr eventP{event};
    event = nullptr;

    // Attach the bus to sd_event to service user requests
    bus.attach_event(eventP.get(), SD_EVENT_PRIORITY_NORMAL);

    sdbusplus::server::manager::manager objManager(bus, OCC_CONTROL_ROOT);
#ifdef READ_OCC_SENSORS
    sdbusplus::server::manager::manager objManagerXyz(bus, OCC_SENSORS_ROOT);
#endif
#ifdef POWER10
    sdbusplus::server::manager::manager objManagerXyzControl(
        bus, "/xyz/openbmc_project/control");
#endif
    open_power::occ::Manager mgr(eventP);

    // Claim the bus since all the house keeping is done now
    bus.request_name(OCC_CONTROL_BUSNAME);

    // Wait for requests
    sd_event_loop(eventP.get());

    return 0;
}
