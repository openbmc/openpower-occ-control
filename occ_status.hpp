#pragma once
#include "config.h"

#include "i2c_occ.hpp"
#include "occ_command.hpp"
#include "occ_device.hpp"
#include "occ_events.hpp"
#include "powercap.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <org/open_power/Control/Host/server.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#ifdef POWER10
#include <sdeventplus/event.hpp>
#include <sdeventplus/utility/timer.hpp>
#endif

#include <functional>

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
using instanceID = unsigned int;

// IPMI sensor ID for a given OCC instance
using sensorID = uint8_t;

// Human readable sensor name for DBus tree. E.g. "CPU0_OCC"
using sensorName = std::string;

// OCC sensors definitions in the map
using sensorDefs = std::tuple<sensorID, sensorName>;

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
     *  @param[in] event    - sd_event unique pointer reference
     *  @param[in] path     - DBus object path
     *  @param[in] manager  - OCC manager instance
     *  @param[in] callBack - Callback handler to invoke during
     *                        property change
     *  @param[in] resetCallBack - callback handler to invoke for resetting the
     *                             OCC if PLDM is the host communication
     *                             protocol
     */
    Status(EventPtr& event, const char* path, Manager& managerRef,
#ifdef POWER10
           std::unique_ptr<powermode::PowerMode>& powerModeRef,
#endif
           std::function<void(instanceID, bool)> callBack = nullptr
#ifdef PLDM
           ,
           std::function<void(instanceID)> resetCallBack = nullptr
#endif
           ) :

        Interface(utils::getBus(), getDbusPath(path).c_str(),
                  Interface::action::defer_emit),
        path(path), managerCallBack(callBack), instance(getInstance(path)),
        manager(managerRef),
#ifdef POWER10
        pmode(powerModeRef),
#endif
        device(event,
#ifdef I2C_OCC
               fs::path(DEV_PATH) / i2c_occ::getI2cDeviceName(path),
#else
               fs::path(DEV_PATH) /
                   fs::path(sysfsName + "." + std::to_string(instance + 1)),
#endif
               managerRef, *this,
#ifdef POWER10
               powerModeRef,
#endif
               instance),
        hostControlSignal(
            utils::getBus(),
            sdbusRule::type::signal() + sdbusRule::member("CommandComplete") +
                sdbusRule::path("/org/open_power/control/host0") +
                sdbusRule::interface("org.open_power.Control.Host") +
                sdbusRule::argN(0, Control::convertForMessage(
                                       Control::Host::Command::OCCReset)),
            std::bind(std::mem_fn(&Status::hostControlEvent), this,
                      std::placeholders::_1)),
        occCmd(instance, (fs::path(OCC_CONTROL_ROOT) /
                          (std::string(OCC_NAME) + std::to_string(instance)))
                             .c_str())
#ifdef POWER10
        ,
        sdpEvent(sdeventplus::Event::get_default()),
        safeStateDelayTimer(
            sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>(
                sdpEvent, std::bind(&Status::safeStateDelayExpired, this))),
        occReadStateFailTimer(
            sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>(
                sdpEvent, std::bind(&Status::occReadStateNow, this)))
#endif

#ifdef PLDM
        ,
        resetCallBack(resetCallBack)
#endif
    {
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

    /** @brief Gets the occ instance number */
    unsigned int getOccInstanceID()
    {
        return instance;
    }

    /** @brief Is this OCC the master OCC */
    bool isMasterOcc()
    {
        return device.master();
    }

    /** @brief Read OCC state (will trigger kernel to poll the OCC) */
    void readOccState();

    /** @brief Called when device errors are detected */
    void deviceError();

#ifdef POWER10
    /** @brief Handle additional tasks when the OCCs reach active state */
    void occsWentActive();

    /** @brief Send Ambient & Altitude data to OCC
     *
     *  @param[in] ambient - temperature to send (0xFF will force read
     *                       of current temperature and altitude)
     *  @param[in] altitude - altitude to send (0xFFFF = unavailable)
     *
     *  @return SUCCESS on success
     */
    CmdStatus sendAmbient(const uint8_t ambient = 0xFF,
                          const uint16_t altitude = 0xFFFF);
#endif // POWER10

    /** @brief Return the HWMON path for this OCC
     *
     *  @return path or empty path if not found
     */
    fs::path getHwmonPath();

  private:
    /** @brief OCC dbus object path */
    std::string path;

    /** @brief Callback handler to be invoked during property change.
     *         This is a handler in Manager class
     */
    std::function<void(instanceID, bool)> managerCallBack;

    /** @brief OCC instance number. Ex, 0,1, etc */
    unsigned int instance;

    /** @brief The last state read from the OCC */
    unsigned int lastState = 0;

    /** @brief Number of retry attempts to open file and update state. */
    const unsigned int occReadRetries = 1;

    /** @brief Current number of retries attempted towards occReadRetries. */
    size_t currentOccReadRetriesCount = 0;

    /** @brief The Trigger to indicate OCC State is valid or not. */
    bool stateValid = false;

    /** @brief OCC instance to Sensor definitions mapping */
    static const std::map<instanceID, sensorDefs> sensorMap;

    /** @brief OCC manager object */
    const Manager& manager;

#ifdef POWER10
    /** @brief OCC PowerMode object */
    std::unique_ptr<powermode::PowerMode>& pmode;
#endif

    /** @brief OCC device object to do bind and unbind */
    Device device;

    /** @brief Subscribe to host control signal
     *
     *  Once the OCC reset is requested, BMC sends that message to host.
     *  If the host does not ack the message, then there would be a timeout
     *  and we need to catch that to log an error
     **/
    sdbusplus::bus::match_t hostControlSignal;

    /** @brief Command object to send commands to the OCC */
    OccCommand occCmd;

#ifdef POWER10
    /** @brief timer event */
    sdeventplus::Event sdpEvent;
#endif

    /** @brief hwmon path for this OCC */
    fs::path hwmonPath;

    /** @brief Callback function on host control signals
     *
     *  @param[in]  msg - Data associated with subscribed signal
     */
    void hostControlEvent(sdbusplus::message::message& msg);

    /** @brief Sends a message to host control command handler to reset OCC
     */
    void resetOCC();

    /** @brief Determines the instance ID by specified object path.
     *  @param[in]  path  Estimated OCC Dbus object path
     *  @return  Instance number
     */
    static int getInstance(const std::string& path)
    {
        return (path.empty() ? 0 : path.back() - '0');
    }

#ifdef POWER10
    /**
     * @brief Timer that is started when OCC is detected to be in safe mode
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>
        safeStateDelayTimer;

    /** @brief Callback for timer that is started when OCC was detected to be in
     * safe mode. Called to verify and then disable and reset the OCCs.
     */
    void safeStateDelayExpired();

    /**
     * @brief Timer that is started when OCC read Valid state failed.
     */
    sdeventplus::utility::Timer<sdeventplus::ClockId::Monotonic>
        occReadStateFailTimer;

#endif // POWER10
    /** @brief Callback for timer that is started when OCC state
     * was not able to be read. Called to attempt another read when needed.
     */
    void occReadStateNow();

    /** @brief Override the sensor name with name from the definition.
     *  @param[in]  estimatedPath - Estimated OCC Dbus object path
     *  @return  Fixed OCC DBus object path
     */
    static std::string getDbusPath(const std::string& estimatedPath)
    {
        if (!estimatedPath.empty())
        {
            auto it = sensorMap.find(getInstance(estimatedPath));
            if (sensorMap.end() != it)
            {
                auto& name = std::get<1>(it->second);
                if (!name.empty() && name != "None")
                {
                    auto objectPath = fs::path(estimatedPath);
                    objectPath.replace_filename(name);
                    return objectPath.string();
                }
            }
        }

        return estimatedPath;
    }
#ifdef PLDM
    std::function<void(instanceID)> resetCallBack = nullptr;
#endif
};

} // namespace occ
} // namespace open_power
