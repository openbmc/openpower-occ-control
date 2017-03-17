#include <phosphor-logging/log.hpp>
#include <exception>
#include "occ_pass_through.hpp"

int main(int argc, char* argv[])
{
    try
    {
        open_power::occ::pass_through::run();
    }
    catch (const std::exception& e)
    {
        using namespace phosphor::logging;
        log<level::ERR>(e.what());
    }

    return 0;
}
