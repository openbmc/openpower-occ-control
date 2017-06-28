#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include "occ_device.hpp"
namespace open_power
{
namespace occ
{

namespace Base = sdbusplus::org::open_power::OCC::server;
using Interface = sdbusplus::server::object::object<Base::Status>;

/** @class Status
 *  @brief Implementation of OCC Active Status
 */
class Status : public Interface
{
    public:
        Status() = delete;
        ~Status() = default;
        Status(const Status&) = delete;
        Status& operator=(const Status&) = delete;
        Status(Status&&) = default;
        Status& operator=(Status&&) = default;

        /** @brief Constructs the Status object
         *
         *  @param[in] bus  - DBus bus to attach to
         *  @param[in] path - DBus object path
         */
        Status(sdbusplus::bus::bus& bus, const char* path)
            : Interface(bus, path),
              path(path),
              device(path)
        {
            // Nothing to do here
        }

        /** @brief SET OccActive to True or False
         *
         *  @param[in] value - Intended value
         *
         *  @return          - Updated value of the property
         */
        bool occActive(bool value) override;

    private:
        /** @brief OCC dbus object path */
        std::string path;

        /** @brief OCC device object to do bind and unbind */
        Device device;
};

} // namespace occ
} // namespace open_power
