#pragma once

#include "occ_status.hpp"

#include <iostream>
#include <sdbusplus/bus/match.hpp>

namespace pldm
{

namespace sdbusRule = sdbusplus::bus::match::rules;

using TerminusID = uint8_t;
using SensorID = uint16_t;
using EntityType = uint16_t;
using EntityInstance = uint16_t;
using SensorOffset = uint8_t;
using EventState = uint8_t;
constexpr open_power::occ::instanceID start = 0;

/** @class Interface
 * 
 *  @brief Abstracts the PLDM details related to the OCC
 */
class Interface
{
  public:
    Interface() = delete;
    ~Interface() = default;
    Interface(const Interface&) = delete;
    Interface& operator=(const Interface&) = delete;
    Interface(Interface&&) = delete;
    Interface& operator=(Interface&&) = delete;

    /** @brief Constructs the PLDM Interface object for OCC functions 
     *
     *  @param[in] bus      - reference to systemd bus
     *  @param[in] callBack - callBack handler to invoke when the OCC state
     *                        changes.
     */
    explicit Interface(sdbusplus::bus::bus& bus,
              std::function<bool(open_power::occ::instanceID, bool)> callBack) :
        bus(bus),
        callBack(callBack),
        pldmEventSignal(
            bus,
            sdbusRule::type::signal() + sdbusRule::member("StateSensorEvent") +
                sdbusRule::path("/xyz/openbmc_project/pldm") +
                sdbusRule::interface("xyz.openbmc_project.PLDM.Event"),
            std::bind(std::mem_fn(&Interface::sensorEvent), this,
                      std::placeholders::_1))
    {
    }

    /** @brief Check if the PDR cache for PLDM OCC sensors is valid
     *
     *  @return true if cache is populated and false if the cache is not
     *          populated.
     */
    auto isPDRCacheValid()
    {
        return (sensorToOCCInstance.empty() ? false : true);
    }

    /** @brief Fetch the OCC state sensor PDRs and populate the PDR cache with
     *         sensorId to OCC instance information.
     */
    void fetchOCCSensorInfo();

    /** @brief When the OCC state changes host sends Platform Event Message
     *         StateSensorEvent, this function processes the D-Bus signal
     *         with the sensor event information.
     *
     *  @param[in] msg - data associated with the subscribed signal
     *
     */
    void sensorEvent(sdbusplus::message::message& msg);

  private:
    /** @brief reference to the systemd bus*/
    sdbusplus::bus::bus& bus;

    /** @brief Callback handler to be invoked when the state of the OCC
     *         changes
     */
    std::function<bool(open_power::occ::instanceID, bool)> callBack;

    /** @brief Used to subscribe to dbus pldm StateSensorEvent signal
     * When the host soft off is complete, it sends an platform event message
     * to BMC's pldmd, and the pldmd will emit the StateSensorEvent signal.
     **/
    sdbusplus::bus::match_t pldmEventSignal;

    /** @brief PLDM Sensor ID to OCC Instance mapping
     */
    std::map<SensorID, open_power::occ::instanceID> sensorToOCCInstance;

    /** @brief Sensor offset of state set ID
     * PLDM_STATE_SET_OPERATIONAL_RUNNING_STATUS in state sensor PDR.
     */
    SensorOffset sensorOffset;
};

} // namespace pldm
