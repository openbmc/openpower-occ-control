#include <phosphor-logging/log.hpp>
#include <exception>
#include "occ_manager.hpp"
#include "config.h"

int main(int argc, char* argv[])
{
    try
    {
        auto bus = sdbusplus::bus::new_default();
        bus.request_name(OCC_CONTROL_BUSNAME);

        sdbusplus::server::manager::manager objManager(bus,
                                                       OCC_CONTROL_ROOT);

        open_power::occ::Manager mgr(bus);

        while (true)
        {
            bus.process_discard();
            bus.wait();
        }
    }
    catch (const std::exception& e)
    {
        using namespace phosphor::logging;
        log<level::ERR>(e.what());
        return -1;
    }

    return 0;
}
