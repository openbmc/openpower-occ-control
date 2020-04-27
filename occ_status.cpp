#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "utils.hpp"

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

using namespace phosphor::logging;

// Dbus call from PLDM/IPMI that leads to reset of OCC
void Status::resetOCCDbus()
{
#ifdef ENABLE_PLDM
    static constexpr auto pldmObjPath = "/xyz/openbmc_project/pldm";
    static constexpr auto pdrInterface = "xyz.openbmc_project.PLDM.PDR";
    static constexpr auto PLDM_OCC_ENTITY_TYPE = 67;
    static constexpr auto PLDM_OCC_RESET_STATE_ID = 192;
    static constexpr auto MCTP_EID = 10;

    uint8_t tid = 0;
    uint8_t mctpEid = MCTP_EID;
    uint16_t entityType = PLDM_OCC_ENTITY_TYPE;
    uint16_t stateSetId = PLDM_OCC_RESET_STATE_ID;
    std::vector<std::vector<uint8_t>> pdrList;

    try
    {
        auto service = getService(bus, pldmObjPath, pdrInterface);
        auto method = bus.new_method_call(service.c_str(), pldmObjPath,
                                          pdrInterface, "FindStateEffecterPDR");
        method.append(tid);
        method.append(entityType);
        method.append(stateSetId);
        auto reply = bus.call(method);
        reply.read(pdrList);
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("FindStateEffecterPDR dbus call returned error, ",
                        entry("ERROR = %d", e.what()));
        return;
    }

    try
    {
        static constexpr auto pldmRequester =
            "xyz.openbmc_project.PLDM.Requester";
        method = bus.new_method_call(service.c_str(), pldmObjPath,
                                     pldmRequester, "GetInstanceId");
        method.append(mctpEid);
        reply = bus.call(method);
        uint8_t instanceId = 0;
        reply.read(instanceId);
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("GetInstanceId dbus call returned error, ",
                        entry("ERROR = %d", e.what()));
        return;
    }
    // entity instance will be the instance of this class incremented by 1.
    // The PLDM entity instance numbering starts from 1 and not 0,
    // whereas this class starts numbering OCC instances from 0
    uint16_t entityInstance = static_cast<uint16_t>(instance) + 1;

    return resetOCCPLDM(pdrList, mctpEid, instanceId, entityInstance);

#else
    constexpr auto CONTROL_HOST_PATH = "/org/open_power/control/host0";
    constexpr auto CONTROL_HOST_INTF = "org.open_power.Control.Host";
    // This will throw exception on failure
    auto service = getService(bus, CONTROL_HOST_PATH, CONTROL_HOST_INTF);

    auto method = bus.new_method_call(service.c_str(), CONTROL_HOST_PATH,
                                      CONTROL_HOST_INTF, "Execute");

    // OCC Reset control command
    method.append(convertForMessage(Control::Host::Command::OCCReset).c_str());

    // OCC Sensor ID for callout reasons
    method.append(std::variant<uint8_t>(std::get<0>(sensorMap.at(instance))));
    bus.call_noreply(method);
    return;

#endif
}

// Handler called by Host control command handler to convey the
// status of the executed command
void Status::hostControlEvent(sdbusplus::message::message& msg)
{
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

// Get state effecter request msg from PDR vector
std::vector<uint8_t> getStateEffecterRequest(pldm_state_effecter_pdr* pdr,
                                             uint8_t instanceId)
{
    static constexpr auto PLDM_OCC_RESET_STATE_VALUE = 3;
    static constexpr auto PLDM_OCC_RESET_STATE_ID = 192;
    auto effecterID = pdr->effecter_id;
    auto compositeEffecterCount = pdr->composite_effecter_count;
    assert(compositeEffecterCount != 0);
    uint16_t stateSetId = PLDM_OCC_RESET_STATE_ID;
    auto possibleStates =
        reinterpret_cast<state_effecter_possible_states*>(pdr->possible_states);
    std::vector<set_effecter_state_field> stateField(
        compositeEffecterCount * sizeof(set_effecter_state_field));

    std::vector<uint8_t> request(sizeof(pldm_msg_hdr) + sizeof(effecterID) +
                                 sizeof(compositeEffecterCount) +
                                 stateField.size());
    auto requestMsg = reinterpret_cast<pldm_msg*>(request.data());
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
        possibleStates += possibleStates->possible_states_size +
                          sizeof(stateSetId) +
                          sizeof(possibleStates->possible_states_size);
    }
    auto rc = encode_set_state_effecter_states_req(
        instanceId, effecterID, compositeEffecterCount, stateField.data(),
        requestMsg);
    if (rc != PLDM_SUCCESS)
    {
        log<level::ERR>("encode set effecter states request returned error ",
                        entry("RC = %d", rc));
    }
    return request;
}

// Get state effecter request msg from PDR vector
uint8_t* pldmSendRecv(std::vector<uint8_t>& request, uint8_t mctpEid,
                      size_t& pdrResponseMsgSize)
{
    int fd = pldm_open();
    FileDescriptor FileDesc(fd);
    if (-1 == fd)
    {
        log<level::ERR>("failed to init mctp");
    }
    uint8_t* pdrResponseMsg = nullptr;
    auto requesterRC =
        pldm_send_recv(mctpEid, fd, request.data(), request.size(),
                       &pdrResponseMsg, &pdrResponseMsgSize);
    if (requesterRC != 0)
    {
        log<level::ERR>("PLDM send receive failed, rc:",
                        entry("RC = %d", requesterRC));
    }
    return pdrResponseMsg;
}

// PLDM sends a message to host control command handler to reset OCC
void resetOCCPLDM(std::vector<std::vector<uint8_t>>& pdrList, uint8_t mctpEid,
                  uint8_t instanceId, uint16_t entityInstance)
{
    for (const auto& pdrVec : pdrList)
    {
        auto pdr = reinterpret_cast<pldm_state_effecter_pdr*>(
            const_cast<uint8_t*>(pdrVec.data()));

        if (pdr->hdr.type == PLDM_STATE_EFFECTER_PDR &&
            pdr->entity_instance == entityInstance)
        {
            auto stateEffecReqMsg = getStateEffecterRequest(pdr, instanceId);
            size_t pdrResponseMsgSize{};
            auto pdrResponseMsg =
                pldmSendRecv(stateEffecReqMsg, mctpEid, pdrResponseMsgSize);
            std::unique_ptr<uint8_t, decltype(std::free)*> pdrResponseMsgPtr{
                pdrResponseMsg, std::free};
            uint8_t completionCode;
            auto response =
                reinterpret_cast<pldm_msg*>(pdrResponseMsgPtr.get());
            auto rc = decode_set_state_effecter_states_resp(
                response, pdrResponseMsgSize - sizeof(pldm_msg_hdr),
                &completionCode);
            if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
            {
                log<level::ERR>(
                    "decode set effecter states request returned error.",
                    entry("RC = %d", rc),
                    entry("COMPLETION_CODE = %d", completionCode));
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

} // namespace occ
} // namespace open_power
