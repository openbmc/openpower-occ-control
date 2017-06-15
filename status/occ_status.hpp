#pragma once
#include <tuple>
#include <systemd/sd-bus.h>
#include <sdbusplus/server.hpp>
#include <org/open_power/OCC/Status/server.hpp>

namespace org
{
namespace open_power
{
namespace occ
{
namespace status
{

using Iface = sdbusplus::server::object::object<
    sdbusplus::org::open_power::OCC::server::Status>;

/** class Status
 *  @brief implementation of OCC Active status
 */
class Status : public Iface
{
    public:
        /* Define all of the basic class operations:
         *     Not allowed:
         *         - Default constructor to avoid nullptrs.
         *         - Copy operations due to internal unique_ptr.
         *         - Move operations due to 'this' being registered as the
         *           'context' with sdbus.
         *     Allowed:
         *         - Destructor.
         */
        Status() = delete;
        Status(const Status&) = delete;
        Status& operator=(const Status&) = delete;
        Status(Status&&) = delete;
        Status& operator=(Status&&) = delete;
        virtual ~Status() = default;

        /** @brief Constructor to put object onto bus at a dbus path.
         *  @param[in] bus - Bus to attach to.
         *  @param[in] path - Path to attach at.
         */
        Status(sdbusplus::bus::bus& bus, const char* path):Iface(bus,path)
        { }
};


} // namespace status
} // namespace OCC
} // namespace open_power
} // namespace org

