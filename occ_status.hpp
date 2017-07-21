#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include "occ_events.hpp"
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

        /** @brief Constructs the Status object and
         *         the underlying device object
         *
         *  @param[in] bus  - DBus bus to attach to
         *  @param[in] path - DBus object path
         */
        Status(sdbusplus::bus::bus& bus, EventPtr& event, const char* path)
            : Interface(bus, path),
              path(path),
              instance(((this->path.back() - '0'))),
              device(event,
                     name + std::to_string(instance + 1),
                     std::bind(&Status::deviceErrorHandler, this))
        {
            // Nothing to do here
        }

        /** @brief Since we are overriding the setter-occActive but not the
         *         getter-occActive, we need to have this using in order to
         *         allow passthrough usage of the getter-occActive
         */
        using Base::Status::occActive;

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

        /** @brief occ name prefix */
        std::string name = OCC_NAME;

        /** @brief OCC instance number. Ex, 0,1, etc */
        int instance;

        /** @brief OCC instance to Sensor ID mapping */
        static const std::map<int, uint8_t> sensorMap;

        /** @brief OCC device object to do bind and unbind */
        Device device;

        /** @brief Callback handler when device errors are detected */
        void deviceErrorHandler();
};

} // namespace occ
} // namespace open_power
