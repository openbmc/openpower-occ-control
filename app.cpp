#include <phosphor-logging/log.hpp>
#include <exception>
#include "occ_pass_through.hpp"

int main(int argc, char* argv[])
{
    try
    {
        auto bus = sdbusplus::bus::new_default();
        sdbusplus::server::manager::manager objManager(bus,
                                                       OCC_PASS_THROUGH_ROOT);

        open_power::occ::pass_through::manager::Manager mgr(bus);

        bus.request_name(OCC_PASS_THROUGH_BUSNAME);

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
