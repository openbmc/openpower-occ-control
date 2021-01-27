#include "config.h"

#include "occ_manager.hpp"

#include "i2c_occ.hpp"
#include "occ_finder.hpp"
#include "utils.hpp"

#include <experimental/filesystem>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

void Manager::findAndCreateObjects()
{
    for (auto id = 0; id < MAX_CPUS; ++id)
    {
        // Create one occ per cpu
        auto occ = std::string(OCC_NAME) + std::to_string(id);
        createObjects(occ);
    }
}

int Manager::cpuCreated(sdbusplus::message::message& msg)
{
    namespace fs = std::experimental::filesystem;

    sdbusplus::message::object_path o;
    msg.read(o);
    fs::path cpuPath(std::string(std::move(o)));

    auto name = cpuPath.filename().string();
    auto index = name.find(CPU_NAME);
    name.replace(index, std::strlen(CPU_NAME), OCC_NAME);

    createObjects(name);

    return 0;
}

void Manager::createObjects(const std::string& occ)
{
    auto path = fs::path(OCC_CONTROL_ROOT) / occ;

    passThroughObjects.emplace_back(
        std::make_unique<PassThrough>(bus, path.c_str()));

    statusObjects.emplace_back(std::make_unique<Status>(
        bus, event, path.c_str(), *this,
        std::bind(std::mem_fn(&Manager::statusCallBack), this,
                  std::placeholders::_1)
#ifdef PLDM
            ,
        std::bind(std::mem_fn(&pldm::Interface::resetOCC), pldmHandle.get(),
                  std::placeholders::_1)
#endif
            ));

    // Create the power cap monitor object for master occ (0)
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
            bus, *statusObjects.front());
    }
}

void Manager::statusCallBack(bool status)
{
    using namespace phosphor::logging;
    using InternalFailure =
        sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

    // At this time, it won't happen but keeping it
    // here just in case something changes in the future
    if ((activeCount == 0) && (!status))
    {
        log<level::ERR>("Invalid update on OCCActive");
        elog<InternalFailure>();
    }

    activeCount += status ? 1 : -1;

    // Only start presence detection if all the OCCs are bound
    if (activeCount == statusObjects.size())
    {
        for (auto& obj : statusObjects)
        {
            obj->addPresenceWatchMaster();
        }
    }
}

#ifdef I2C_OCC
void Manager::initStatusObjects()
{
    // Make sure we have a valid path string
    static_assert(sizeof(DEV_PATH) != 0);

    auto deviceNames = i2c_occ::getOccHwmonDevices(DEV_PATH);
    auto occMasterName = deviceNames.front();
    for (auto& name : deviceNames)
    {
        i2c_occ::i2cToDbus(name);
        name = std::string(OCC_NAME) + '_' + name;
        auto path = fs::path(OCC_CONTROL_ROOT) / name;
        statusObjects.emplace_back(
            std::make_unique<Status>(bus, event, path.c_str(), *this));
    }
    // The first device is master occ
    pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
        bus, *statusObjects.front(), occMasterName);
}
#endif

#ifdef PLDM
bool Manager::updateOCCActive(instanceID instance, bool status)
{
    return (statusObjects[instance])->occActive(status);
}
#endif


// Send command to specified OCC instance and return response data
//
// instance is the OCC instance: 0xFF for master, 0 = first OCC, 1 = second OCC, ...
// command is vector: [0] = command, [1-2] = data length, [3-] = data
// response contains full OCC response packet (excluding checksum)
//          [0] = seq, [1] = cmd, [2] = status, [3-4] = data length, [5-] = data
//
// Returns SUCCESS when valid response was received
CmdStatus Manager::sendOccCommand(const uint8_t instance,
                                  const std::vector<std::int32_t> &command,
                                  std::vector<std::int32_t> &response) const
{
    using namespace phosphor::logging;
    CmdStatus status = FAILURE;
    response.clear();

    uint8_t target_instance = instance;
    if (instance == 0xFF)
    {
        // Determine master and use that - first instance is always master
        target_instance = 0;
    }

    if ((target_instance < activeCount) &&
        (command.size() >= 3) &&
        (statusObjects[target_instance]->occ_pt != nullptr))
    {
        status = statusObjects[target_instance]->occ_pt->send(command, response);
        if (status == SUCCESS)
        {
            if (response.size() >= 7)
            {
                // Strip off 2 byte checksum
                response.pop_back();
                response.pop_back();
            }
            else
            {
                log<level::ERR>(fmt::format("Manager::sendOccCommand: Invalid command response ({} bytes)",
                                            response.size()).c_str());
                status = FAILURE;
            }
        }
        else
        {
            if (status == OPEN_FAILURE)
            {
                log<level::WARNING>("Manager::sendOccCommand: Ignoring request - OCC currently not active");
            }
            else
            {
                log<level::ERR>(fmt::format("Manager::sendOccCommand: Send of command failed with status {}",
                                            status).c_str());
            }
        }
    }
    else
    {
        log<level::ERR>(fmt::format("Manager::sendOccCommand: Send of command failed with status {}",
                                    status).c_str());
    }

    return status;
}


} // namespace occ
} // namespace open_power
