#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "utils.hpp"

#include <libpldm/platform.h>
#include <libpldm/pldm.h>

#include <iostream>
#include <phosphor-logging/log.hpp>
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
    static constexpr auto pldmObjPath = "/xyz/openbmc_project/PLDM";
    static constexpr auto pdrInterface = "xyz.openbmc_project.PLDM.Pdr";

    uint8_t tid = 0;
    uint16_t entityInstance = 1;
    uint16_t containerId = 0;
    uint16_t entityType = 67;
    uint16_t stateSetId = 192;
    uint8_t mctp_eid = 10;
    std::vector<uint8_t> pdrVec;

    try
    {
        auto service = getService(bus, pldmObjPath, pdrInterface);
        auto method = bus.new_method_call(service.c_str(), pldmObjPath,
                                          pdrInterface, "FindStateEffecterPDR");
        method.append(tid);
        method.append(containerId);
        method.append(entityType);
        method.append(entityInstance);
        method.append(stateSetId);
        auto reply = bus.call(method);
        reply.read(pdrVec);

        pldm_state_effecter_pdr* pdr =
            reinterpret_cast<pldm_state_effecter_pdr*>(pdrVec.data());

        if (pdr->hdr.type == PLDM_STATE_EFFECTER_PDR)
        {
            uint16_t effecterID = pdr->effecter_id;
            uint8_t compositeEffecterCount = pdr->composite_effecter_count;
            state_effecter_possible_states* possibleStates =
                reinterpret_cast<state_effecter_possible_states*>(
                    pdr->possible_states);
            std::vector<set_effecter_state_field> stateField(
                compositeEffecterCount, {0, 0});

            std::vector<uint8_t> stateEffecReqMsg(
                sizeof(pldm_msg_hdr) +
                PLDM_SET_STATE_EFFECTER_STATES_REQ_BYTES);
            auto stateEffecReq =
                reinterpret_cast<pldm_msg*>(stateEffecReqMsg.data());

            uint8_t* pdrResponseMsg = nullptr;
            size_t pdrResponseMsgSize{};

            for (uint8_t effecters = 0x01; effecters <= compositeEffecterCount;
                 effecters++)
            {
                if (possibleStates->state_set_id == stateSetId &&
                    possibleStates->possible_states_size != 0x00)
                {
                    stateField[effecters] = {PLDM_REQUEST_SET, 3};

                    try
                    {
                        static constexpr auto pldmRequester =
                            "xyz.openbmc_project.PLDM.Requester";
                        service = getService(bus, pldmObjPath, pldmRequester);
                        method =
                            bus.new_method_call(service.c_str(), pldmObjPath,
                                                pldmRequester, "GetInstanceId");
                        method.append(mctp_eid);
                        reply = bus.call(method);
                        uint8_t instanceId = 0;
                        reply.read(instanceId);
                        auto rc = encode_set_state_effecter_states_req(
                            instanceId, effecterID, compositeEffecterCount,
                            stateField.data(), stateEffecReq);
                        if (rc != PLDM_SUCCESS)
                        {
                            std::cerr << "encode set effecter states request "
                                         "returned error, with rc: "
                                      << rc << "\n";
                            int fd = pldm_open();
                            if (-1 == fd)
                            {
                                std::cerr << "failed to init mctp "
                                          << "\n";
                                return;
                            }
                            pldm_send_recv(
                                mctp_eid, fd, stateEffecReqMsg.data(),
                                stateEffecReqMsg.size(), &pdrResponseMsg,
                                &pdrResponseMsgSize);
                            std::cout << "Response Message:" << std::endl;
                            std::vector<uint8_t> responseMsg;
                            responseMsg.resize(pdrResponseMsgSize);
                            memcpy(responseMsg.data(), pdrResponseMsg,
                                   responseMsg.size());

                            free(pdrResponseMsg);
                            std::ostringstream tempStream;
                            if (!responseMsg.empty())
                            {
                                for (int byte : responseMsg)
                                {
                                    tempStream << std::hex << byte << " ";
                                }
                            }
                            std::cout << tempStream.str().c_str() << std::endl;
                        }
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "GetInstanceId dbus call returned error "
                                  << ", error = " << e.what() << "\n";
                        return;
                    }
                }
                else
                {
                    possibleStates +=
                        possibleStates->possible_states_size +
                        sizeof(stateSetId) +
                        sizeof(possibleStates->possible_states_size);
                }
            }
        }
        else
        {
            std::cerr << "PDR of type state effecter not found"
                      << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "FindStateEffecterPDR dbus call returned error "
                  << ", error = " << e.what() << "\n";
        return;
    }

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
