#include "occ_status.hpp"

#include "occ_sensor.hpp"
#include "utils.hpp"

#include <libpldm/platform.h>
#include <libpldm/pldm.h>

#include <iomanip>
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
    using namespace phosphor::logging;

#ifdef PLDM
    static constexpr auto pldmObjPath = "/xyz/openbmc_project/pldm";
    static constexpr auto pdrInterface = "xyz.openbmc_project.PLDM.PDR";
    static constexpr auto OCC_RESET_ENTITY_TYPE = 67;
    static constexpr auto OCC_RESET_STATE_ID = 192;
    static constexpr auto OCC_RESET_MCTP_ID = 10;
    static constexpr auto OCC_RESET_STATE = 3;

    uint8_t tid = 0;
    uint16_t entityInstance =
        static_cast<uint16_t>(instance) +
        1; // entity instance will be the instance of this class incremented by
           // 1 as resetOCC is member function of class Status
    uint16_t entityType = OCC_RESET_ENTITY_TYPE;
    uint16_t stateSetId = OCC_RESET_STATE_ID;
    uint8_t mctpEid = OCC_RESET_MCTP_ID;
    std::vector<std::vector<uint8_t>> pdrList;
    pldm_state_effecter_pdr* pdr = {};

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

        for (const auto& pdrVec : pdrList)
        {
            pdr = const_cast<reinterpret_cast<pldm_state_effecter_pdr*>(
                pdrVec.data())>;
            if (pdr->entity_instance == entityInstance)
            {
                break;
            }
        }

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
                    stateField.emplace_back(
                        {PLDM_REQUEST_SET, OCC_RESET_STATE});
                }
                else
                {
                    stateField.emplace_back({PLDM_NO_CHANGE, 0});
                }
                possibleStates += possibleStates->possible_states_size +
                                  sizeof(stateSetId) +
                                  sizeof(possibleStates->possible_states_size);
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
                auto rc = encode_set_state_effecter_states_req(
                    instanceId, effecterID, compositeEffecterCount,
                    stateField.data(), stateEffecReq);
                if (rc != PLDM_SUCCESS)
                {
                    log<level::ERR>("encode set effecter states request "
                                    "returned error ",
                                    entry("RC = %d", rc));
                    return;
                }
                int fd = pldm_open();
                FileDescriptor::FileDescriptor(fd);
                if (-1 == fd)
                {
                    log<level::ERR>("failed to init mctp");
                    return;
                }
                uint8_t* pdrResponseMsg = nullptr;
                size_t pdrResponseMsgSize{};
                auto requesterRC =
                    pldm_send_recv(mctpEid, fd, stateEffecReqMsg.data(),
                                   stateEffecReqMsg.size(), &pdrResponseMsg,
                                   &pdrResponseMsgSize);
                if (requesterRC != 0)
                {
                    log<level::ERR>("PLDM send receive failed, rc:",
                                    entry("RC = %d", requesterRC));
                    return;
                }
                uint8_t completionCode;
                uint8_t hdrSize = 0x3;
                auto response =
                    reinterpret_cast<pldm_msg*>(pdrResponseMsg.data());
                rc = decode_set_state_effecter_states_resp(
                    response, pdrResponseMsgSize - hdrSize, &completionCode);
                if (rc != PLDM_SUCCESS || completionCode != PLDM_SUCCESS)
                {
                    log<level::ERR>(
                        "decode set effecter states request returned "
                        "error, " entry("RC = %d", rc),
                        entry("COMPLETION CODE = %d", completionCode));
                    return;
                }
            }
            catch (const std::exception& e)
            {
                log<level::ERR>("GetInstanceId dbus call returned error, ",
                                entry("ERROR = %d", e.what()));
                return;
            }
        }
        else
        {
            if (pdr->entity_instance != entityInstance)
            {
                log<level::ERR>(
                    "No PDRs were found with, ",
                    entry("ENTITY INSTANCE = %d", OCC_RESET_ENTITY_TYPE));
                return;
            }
            log<level::ERR>("PDR of type state effecter not found.");
            return;
        }
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("FindStateEffecterPDR dbus call returned error, ",
                        entry("ERROR = %d", e.what()));
        return;
    }
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
    method.append(sdbusplus::message::variant<uint8_t>(
        std::get<0>(sensorMap.at(instance))));
    bus.call_noreply(method);
    return;

#endif
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
