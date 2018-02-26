#pragma once

#include <functional>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include <org/open_power/Control/Host/server.hpp>
#include "occ_events.hpp"
#include "occ_device.hpp"
#include "i2c_occ.hpp"

namespace open_power
{
namespace occ
{

class Manager;
namespace Base = sdbusplus::org::open_power::OCC::server;
using Interface = sdbusplus::server::object::object<Base::Status>;

// IPMID's host control application
namespace Control = sdbusplus::org::open_power::Control::server;

// For waiting on signals
namespace sdbusRule = sdbusplus::bus::match::rules;

// OCC status instance. Ex. for "occ0", the instance is 0
using instanceID = int;

// IPMI sensor ID for a given OCC instance
using sensorID = uint8_t;

// OCC sysfs name prefix
const std::string sysfsName = "occ-hwmon";

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
         *  @param[in] bus      - DBus bus to attach to
         *  @param[in] event    - sd_event unique pointer reference
         *  @param[in] path     - DBus object path
         *  @param[in] manager  - OCC manager instance
         *  @param[in] callBack - Callback handler to invoke during
         *                        property change
         */
        Status(sdbusplus::bus::bus& bus,
               EventPtr& event,
               const char* path,
               const Manager& manager,
               std::function<void(bool)> callBack = nullptr)
            : Interface(bus, path, true),
              bus(bus),
              path(path),
              callBack(callBack),
              instance(((this->path.back() - '0'))),
              device(event,
#ifdef I2C_OCC
                     i2c_occ::getI2cDeviceName(path),
#else
                     sysfsName + "." + std::to_string(instance + 1),
#endif
                     manager,
                     *this,
                     std::bind(std::mem_fn(&Status::deviceErrorHandler), this,
                               std::placeholders::_1)),
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
            // Check to see if we have OCC already bound.  If so, just set it
            if (device.bound())
            {
                this->occActive(true);
            }

            // Announce that we are ready
            this->emit_object_added();
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

        /** @brief Starts OCC error detection */
        inline void addErrorWatch()
        {
            return device.addErrorWatch();
        }

        /** @brief Stops OCC error detection */
        inline void removeErrorWatch()
        {
            return device.removeErrorWatch();
        }

        /** @brief Starts to watch how many OCCs are present on the master */
        inline void addPresenceWatchMaster()
        {
            return device.addPresenceWatchMaster();
        }

    private:

        /** @brief sdbus handle */
        sdbusplus::bus::bus& bus;

        /** @brief OCC dbus object path */
        std::string path;

        /** @brief Callback handler to be invoked during property change.
         *         This is a handler in Manager class
         */
        std::function<void(bool)> callBack;

        /** @brief OCC instance number. Ex, 0,1, etc */
        int instance;

        /** @brief OCC instance to Sensor ID mapping */
        static const std::map<instanceID, sensorID> sensorMap;

        /** @brief OCC device object to do bind and unbind */
        Device device;

        /** @brief Subscribe to host control signal
         *
         *  Once the OCC reset is requested, BMC sends that message to host.
         *  If the host does not ack the message, then there would be a timeout
         *  and we need to catch that to log an error
         **/
        sdbusplus::bus::match_t hostControlSignal;

        /** @brief Callback handler when device errors are detected
         *
         *  @param[in]  error - True if an error is reported, false otherwise
         */
        void deviceErrorHandler(bool error);

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
