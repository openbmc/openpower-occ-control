#include <powercap.hpp>
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{
namespace powercap
{

using namespace phosphor::logging;


void PowerCap::pcapChanged(sdbusplus::message::message& msg)
{
    log<level::DEBUG>("Power Cap Change Detected");
    if (!occStatus.occActive())
    {
        // Nothing to  do
        return;
    }
    // TODO - Process this change
}

} // namespace open_power

} // namespace occ

}// namespace powercap
