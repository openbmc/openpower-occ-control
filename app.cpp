#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <org/open_power/OCC/Device/error.hpp>
#include "occ_manager.hpp"
#include "occ_events.hpp"
#include "phosphor-logging/elog-errors.hpp"
#include "config.h"

using namespace phosphor::logging;

using namespace sdbusplus::org::open_power::OCC::Device::Error;
using InternalFailure = sdbusplus::xyz::openbmc_project::Common::
                                Error::InternalFailure;

int main(int argc, char* argv[])
{
    auto bus = sdbusplus::bus::new_default();

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
    open_power::occ::Manager mgr(bus, eventP);

    // Claim the bus since all the house keeping is done now
    bus.request_name(OCC_CONTROL_BUSNAME);

    // Wait for requests
    sd_event_loop(eventP.get());

    return 0;
}
