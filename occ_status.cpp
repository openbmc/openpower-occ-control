#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "utils.hpp"

#include <pldm/libpldm/requester/pldm.h>

#include <phosphor-logging/log.hpp>
#include <pldm/tool/pldm_cmd_helper.hpp>
namespace open_power
{
namespace occ
{

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value != this->occActive())
    {
        if (value)
        {
            // Bind the device
            device.bind();

            // Start watching for errors
            addErrorWatch();

            // Call into Manager to let know that we have bound
            if (this->callBack)
            {
                this->callBack(value);
            }
        }
        else
        {
            // Call into Manager to let know that we will unbind.
            if (this->callBack)
            {
                this->callBack(value);
            }

            // Stop watching for errors
            removeErrorWatch();

            // Do the unbind.
            device.unBind();
        }
    }
    else if (value && !device.bound())
    {
        // Existing error watch is on a dead file descriptor.
        removeErrorWatch();

        /*
         * In it's constructor, Status checks Device::bound() to see if OCC is
         * active or not.
         * Device::bound() checks for occX-dev0 directory.
         * We will lose occX-dev0 directories during FSI rescan.
         * So, if we start this application (and construct Status), and then
         * later do FSI rescan, we will end up with occActive = true and device
         * NOT bound. Lets correct that situation here.
         */
        device.bind();

        // Add error watch again
        addErrorWatch();
    }
    else if (!value && device.bound())
    {
        removeErrorWatch();

        // In the event that the application never receives the active signal
        // even though the OCC is active (this can occur if the BMC is rebooted
        // with the host on, since the initial OCC driver probe will discover
        // the OCCs), this application needs to be able to unbind the device
        // when we get the OCC inactive signal.
        device.unBind();
    }
    return Base::Status::occActive(value);
}

// Callback handler when a device error is reported.
void Status::deviceErrorHandler(bool error)
{
    // Make sure we have an error
    if (error)
    {
        // This would deem OCC inactive
        this->occActive(false);

        // Reset the OCC
        this->resetOCC();
    }
}

// Sends message to host control command handler to reset OCC
void Status::resetOCC()
{
    // IF BUILD PLDM
    dbus_api::Pdr p;
    dbus_api::Requester requester;
    this->instance = 1;
    uint16_t containerId = 0;
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    mctp_eid_t mctp_eid = 10;
    uint8_t instanceId;

    auto pdrVec =
        p.findStateEffecterPDR(containerId, entityType, instance, stateSetId);
    pldm_state_effecter_pdr* pdr =
        reinterpret_cast<pldm_state_effecter_pdr*>(pdrVec.data());

    if (pdr->hdr.type == PLDM_STATE_EFFECTER_PDR)
    {
        uint16_t effecterID = pdr->effecter_id;
        uint8_t compositeEffecterCount = pdr->composite_effecter_count;
        state_effecter_possible_states* possibleStates =
            reinterpret_cast<state_effecter_possible_states*>(
                pdr->possible_states);

        std::vector<uint8_t> setStateEffecReqMsg(
            sizeof(pldm_msg_hdr) + PLDM_SET_STATE_EFFECTER_STATES_REQ_BYTES);
        auto setStateEffecReq =
            reinterpret_cast<pldm_msg*>(setStateEffecReqMsg.data());

        uint8_t* pdrResponseMsg = nullptr;
        size_t pdrResponseMsgSize{};

        for (uint8_t effecters = 0x01; effecters <= compositeEffecterCount;
             effecters++)
        {
            if (possibleStates->state_set_id == stateSetId)
            {
                set_effecter_state_field stateField =
                    reinterpret_cast<set_effecter_state_field*>(
                        possibleStates->states);
                stateField->set_request = PLDM_REQUEST_SET;
                stateField->effecter_state = 3;
                instanceId = requester.getInstanceId(mctp_eid);
                auto rc = encode_set_state_effecter_states_req(
                    instance_Id, effecterID, compositeEffecterCount, stateField,
                    setStateEffecReq);
                if (rc != PLDM_SUCCESS)
                {
                    int fd = pldm_open();
                    if (-1 == fd)
                    {
                        std::cerr << "failed to init mctp "
                                  << "\n";
                        return -1;
                    }
                    pldm_send_recv(mctp_eid, fd, setStateEffecReqMsg.data(),
                                   setStateEffecReqMsg.size(), &pdrResponseMsg,
                                   &pdrResponseMsgSize);
                    Logger(pldmVerbose, "Response Message:", "");
                    std::vector<uint8_t> responseMsg;
                    responseMsg.resize(pdrResponseMsgSize);
                    memcpy(responseMsg.data(), pdrResponseMsg,
                           responseMsg.size());

                    free(pdrResponseMsg);
                    printBuffer(responseMsg, pldmVerbose);
                }
            }
            else
            {
                possibleStates += possibleStatesSize + sizeof(stateSetID) +
                                  sizeof(possibleStatesSize);
            }
        }
    }
    return;
    // IF BUILD IPMI

    using namespace phosphor::logging;
    constexpr auto CONTROL_HOST_PATH = "/org/open_power/control/host0";
    constexpr auto CONTROL_HOST_INTF = "org.open_power.Control.Host";

    // This will throw exception on failure
    auto service = getService(bus, CONTROL_HOST_PATH, CONTROL_HOST_INTF);

    auto method = bus.new_method_call(service.c_str(), CONTROL_HOST_PATH,
                                      CONTROL_HOST_INTF, "Execute");
    // OCC Reset control command
    method.append(convertForMessage(Control::Host::Command::OCCReset).c_str());

    // OCC Sensor ID for callout reasons
    method.append(sdbusplus::message::variant<uint8_t>(
        std::get<0>(sensorMap.at(instance))));
    bus.call_noreply(method);
    return;
}

// Handler called by Host control command handler to convey the
// status of the executed command
void Status::hostControlEvent(sdbusplus::message::message& msg)
{
    using namespace phosphor::logging;

    std::string cmdCompleted{};
    std::string cmdStatus{};

    msg.read(cmdCompleted, cmdStatus);

    log<level::DEBUG>("Host control signal values",
                      entry("COMMAND=%s", cmdCompleted.c_str()),
                      entry("STATUS=%s", cmdStatus.c_str()));

    if (Control::Host::convertResultFromString(cmdStatus) !=
        Control::Host::Result::Success)
    {
        if (Control::Host::convertCommandFromString(cmdCompleted) ==
            Control::Host::Command::OCCReset)
        {
            // Must be a Timeout. Log an Error trace
            log<level::ERR>(
                "Error resetting the OCC.", entry("PATH=%s", path.c_str()),
                entry("SENSORID=0x%X", std::get<0>(sensorMap.at(instance))));
        }
    }
    return;
}

} // namespace occ
} // namespace open_power
