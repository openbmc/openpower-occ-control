#pragma once

#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include <org/open_power/Control/Host/server.hpp>
#include "occ_events.hpp"
#include "occ_device.hpp"
namespace open_power
{
namespace occ
{

namespace Base = sdbusplus::org::open_power::OCC::server;
using Interface = sdbusplus::server::object::object<Base::Status>;

// IPMID's host control application
namespace Control = sdbusplus::org::open_power::Control::server;

// For waiting on signals
namespace sdbusRule = sdbusplus::bus::match::rules;

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
              bus(bus),
              path(path),
              instance(((this->path.back() - '0'))),
              device(event,
                     name + std::to_string(instance + 1),
                     std::bind(&Status::deviceErrorHandler, this)),
              hostControlSignal(
                     bus,
                     sdbusRule::type::signal() +
                     sdbusRule::member("CommandComplete") +
                     sdbusRule::path("/org/open_power/control/host0") +
                     sdbusRule::interface("org.open_power.Control.Host") +
                     sdbusRule::argN(0, Control::convertForMessage(
                             Control::Host::Command::OCCReset)),
                     std::bind(std::mem_fn(&Status::hostControlEvent),
                            this, std::placeholders::_1))
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

        /** @brief sdbus handle */
        sdbusplus::bus::bus& bus;

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

        /** @brief Subscribe to host control signal
         *
         *  Once the OCC reset is requested, BMC sends that message to host.
         *  If the host does not ack the message, then there would be a timeout
         *  and we need to catch that to log an error
         **/
        sdbusplus::bus::match_t hostControlSignal;

        /** @brief Callback handler when device errors are detected */
        void deviceErrorHandler();

        /** @brief Callback function on host control signals
         *
         *  @param[in]  msg - Data associated with subscribed signal
         */
        void hostControlEvent(sdbusplus::message::message& msg);

        /** @brief Sends a message to host control command handler to reset OCC
         */
        void resetOCC();
};

} // namespace occ
} // namespace open_power
