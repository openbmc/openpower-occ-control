#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/Performance/Mode/server.hpp>
namespace open_power
{
namespace occ
{
namespace performance
{

namespace Base = sdbusplus::org::open_power::OCC::Performance::server;
using Interface = sdbusplus::server::object::object<Base::Mode>;

/** @class Mode
 *  @brief Implementation of OCC performance mode
 */
class Mode : public Interface
{
    public:
        Mode() = delete;
        ~Mode() = default;
        Mode(const Mode&) = delete;
        Mode& operator=(const Mode&) = delete;
        Mode(Mode&&) = default;
        Mode& operator=(Mode&&) = default;

        /** @brief Constructs the Mode object
         *
         *  @param[in] bus  - DBus bus to attach to
         *  @param[in] path - DBus object path
         */
        Mode(sdbusplus::bus::bus& bus, const char* path)
            : Interface(bus, path)
        {
            // Nothing to do here
        }

        /** @brief SET performance mode to True or False
         *
         *  @param[in] value - Intended value
         *
         *  @return          - Updated value of the property
         */
        bool mode(bool value) override;
};

} // namespace performance
} // namespace occ
} // namespace open_power
