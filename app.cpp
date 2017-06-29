#include <phosphor-logging/log.hpp>
#include <exception>
#include "occ_manager.hpp"
#include "occ_events.hpp"
#include "config.h"

using namespace phosphor::logging;

int main(int argc, char* argv[])
{
    try
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

        sdbusplus::server::manager::manager objManager(bus,
                                                       OCC_CONTROL_ROOT);
        open_power::occ::Manager mgr(bus, eventP);

        // Claim the bus since all the house keeping is done now
        bus.request_name(OCC_CONTROL_BUSNAME);

        // Wait for requests
        sd_event_loop(eventP.get());
    }
    catch (const std::exception& e)
    {
        using namespace phosphor::logging;
        log<level::ERR>(e.what());
        return -1;
    }

    return 0;
}
