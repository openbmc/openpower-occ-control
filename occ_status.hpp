#pragma once

#include "file.hpp"
#include "i2c_occ.hpp"
#include "occ_device.hpp"
#include "occ_events.hpp"

#include <libpldm/platform.h>
#include <libpldm/pldm.h>

#include <cassert>
#include <functional>
#include <org/open_power/Control/Host/server.hpp>
#include <org/open_power/OCC/Status/server.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>

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
     *  @param[in] bus      - DBus bus to attach to
     *  @param[in] event    - sd_event unique pointer reference
     *  @param[in] path     - DBus object path
     *  @param[in] manager  - OCC manager instance
     *  @param[in] callBack - Callback handler to invoke during
     *                        property change
     */
    Status(sdbusplus::bus::bus& bus, EventPtr& event, const char* path,
           const Manager& manager,
           std::function<void(bool)> callBack = nullptr) :
        Interface(bus, getDbusPath(path).c_str(), true),
        bus(bus), path(path), callBack(callBack), instance(getInstance(path)),
        device(event,
#ifdef I2C_OCC
               fs::path(DEV_PATH) / i2c_occ::getI2cDeviceName(path),
#else
               fs::path(DEV_PATH) /
                   fs::path(sysfsName + "." + std::to_string(instance + 1)),
#endif
               manager, *this,
               std::bind(std::mem_fn(&Status::deviceErrorHandler), this,
                         std::placeholders::_1)),
        hostControlSignal(
            bus,
            sdbusRule::type::signal() + sdbusRule::member("CommandComplete") +
                sdbusRule::path("/org/open_power/control/host0") +
                sdbusRule::interface("org.open_power.Control.Host") +
                sdbusRule::argN(0, Control::convertForMessage(
                                       Control::Host::Command::OCCReset)),
            std::bind(std::mem_fn(&Status::hostControlEvent), this,
                      std::placeholders::_1))
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

    /** @brief OCC instance to Sensor definitions mapping */
    static const std::map<instanceID, sensorDefs> sensorMap;

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

    /** @brief PLDM sends a message to host control command handler to reset OCC
     */
    template <class PLDMInterface>
    void resetOCCPLDM(PLDMInterface* intf,
                      std::vector<std::vector<uint8_t>>& pdrList,
                      uint8_t mctpEid, uint8_t instanceId, uint16_t stateSetId)
    {
        using namespace phosphor::logging;

        // entity instance will be the instance of this class incremented by 1.
        // The PLDM entity instance numbering starts from 1 and not 0.
        // whereas this class starts numbering OCC instances from 0
        uint16_t entityInstance = static_cast<uint16_t>(instance) + 1;
        static constexpr auto PLDM_OCC_RESET_STATE_VALUE = 3;

        for (const auto& pdrVec : pdrList)
        {
            auto pdr = reinterpret_cast<pldm_state_effecter_pdr*>(
                const_cast<uint8_t*>(pdrVec.data()));

            if (pdr->hdr.type == PLDM_STATE_EFFECTER_PDR &&
                pdr->entity_instance == entityInstance)
            {
                auto effecterID = pdr->effecter_id;
                auto compositeEffecterCount = pdr->composite_effecter_count;
                assert(compositeEffecterCount != 0);
                auto possibleStates =
                    reinterpret_cast<state_effecter_possible_states*>(
                        pdr->possible_states);
                std::vector<set_effecter_state_field> stateField(
                    compositeEffecterCount * sizeof(set_effecter_state_field));

                std::vector<uint8_t> stateEffecReqMsg(
                    sizeof(pldm_msg_hdr) + sizeof(effecterID) +
                    sizeof(compositeEffecterCount) + stateField.size());
                auto stateEffecReq =
                    reinterpret_cast<pldm_msg*>(stateEffecReqMsg.data());

                for (uint8_t effecters = 1; effecters <= compositeEffecterCount;
                     effecters++)
                {
                    if (possibleStates->state_set_id == stateSetId)
                    {
                        stateField.emplace_back(set_effecter_state_field{
                            PLDM_REQUEST_SET, PLDM_OCC_RESET_STATE_VALUE});
                    }
                    else
                    {
                        stateField.emplace_back(
                            set_effecter_state_field{PLDM_NO_CHANGE, 0});
                    }
                    possibleStates +=
                        possibleStates->possible_states_size +
                        sizeof(stateSetId) +
                        sizeof(possibleStates->possible_states_size);
                }
                auto rc = encode_set_state_effecter_states_req(
                    instanceId, effecterID, compositeEffecterCount,
                    stateField.data(), stateEffecReq);
                if (rc != PLDM_SUCCESS)
                {
                    log<level::ERR>(
                        "encode set effecter states request returned error ",
                        entry("RC = %d", rc));
                    return;
                }
                int fd = pldm_open();
                FileDescriptor FileDesc(fd);
                if (-1 == fd)
                {
                    log<level::ERR>("failed to init mctp");
                    return;
                }
                uint8_t* pdrResponseMsg = nullptr;
                size_t pdrResponseMsgSize{};
                auto requesterRC =
                    pldmSendRecv(mctpEid, fd, stateEffecReqMsg.data(),
                                 stateEffecReqMsg.size(), &pdrResponseMsg,
                                 &pdrResponseMsgSize);
                std::unique_ptr<uint8_t, decltype(std::free)*>
                    pdrResponseMsgPtr{pdrResponseMsg, std::free};
                if (requesterRC != 0)
                {
                    log<level::ERR>("PLDM send receive failed, rc:",
                                    entry("RC = %d", requesterRC));
                    return;
                }
                uint8_t completionCode;
                auto response =
                    reinterpret_cast<pldm_msg*>(pdrResponseMsgPtr.get());
                rc = decode_set_state_effecter_states_resp(
                    response, pdrResponseMsgSize - sizeof(pldm_msg_hdr),
                    &completionCode);
                if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
                {
                    log<level::ERR>(
                        "decode set effecter states request returned error.",
                        entry("RC = %d", rc),
                        entry("COMPLETION_CODE = %d", completionCode));
                    return;
                }
            }
            else
            {
                if (pdrVec == pdrList[pdrList.size() - 1])
                {
                    log<level::ERR>("PDR of type state effecter not found.");
                    return;
                }
            }
        }
        return;
    }

    /** @brief Dbus call from PLDM/IPMI that leads to reset of OCC
     */
    void resetOCCDbus();

    /** @brief Wrapper function for pldm_send_recv defined in PLDM
     */
    uint8_t pldmSendRecv(mctp_eid_t eid, int mctp_fd,
                         const uint8_t* pldm_req_msg, size_t req_msg_len,
                         uint8_t** pldm_resp_msg, size_t* resp_msg_len)
    {
        return pldm_send_recv(eid, mctp_fd, pldm_req_msg, req_msg_len,
                              pldm_resp_msg, resp_msg_len);
    }

    /** @brief Sends a message to host control command handler to reset OCC
     */
    void resetOCC()
    {
        resetOCCDbus();
        return;
    }

    /** @brief Determines the instance ID by specified object path.
     *  @param[in]  path  Estimated OCC Dbus object path
     *  @return  Instance number
     */
    static int getInstance(const std::string& path)
    {
        return (path.empty() ? 0 : path.back() - '0');
    }

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
                    auto path = fs::path(estimatedPath);
                    path.replace_filename(name);
                    return path.string();
                }
            }
        }

        return estimatedPath;
    }
};

} // namespace occ
} // namespace open_power
