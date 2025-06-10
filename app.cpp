#include "config.h"

#include "occ_events.hpp"
#include "occ_manager.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

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
        lg2::error("Error creating a default sd_event handler");
        return r;
    }
    open_power::occ::EventPtr eventP{event};
    event = nullptr;

    // Attach the bus to sd_event to service user requests
    bus.attach_event(eventP.get(), SD_EVENT_PRIORITY_NORMAL);

    // Add object manager interfaces (for mapper)
    sdbusplus::server::manager_t objManager(bus, OCC_CONTROL_ROOT);

    sdbusplus::server::manager_t objManagerXyz(bus, OCC_SENSORS_ROOT);

    sdbusplus::server::manager_t objManagerXyzControl(
        bus, "/xyz/openbmc_project/control");

    sdbusplus::server::manager_t objManagerXyzInventory(
        bus, "/xyz/openbmc_project/inventory");
    open_power::occ::Manager mgr(eventP);
    mgr.createPldmHandle();

    // Claim the bus since all the house keeping is done now
    bus.request_name(OCC_CONTROL_BUSNAME);

    // Wait for requests
    sd_event_loop(eventP.get());

    return 0;
}
