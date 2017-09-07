#include <experimental/filesystem>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include "occ_finder.hpp"
#include "occ_manager.hpp"
#include "i2c_occ.hpp"
#include "utils.hpp"
#include "config.h"

namespace open_power
{
namespace occ
{

void Manager::findAndCreateObjects()
{
    // Check if CPU inventory exists already.
    auto occs = open_power::occ::finder::get(bus);
    if (occs.empty())
    {
        // Need to watch for CPU inventory creation.
        for (auto id = 0; id < MAX_CPUS; ++id)
        {
            auto path = std::string(CPU_PATH) + std::to_string(id);
            cpuMatches.emplace_back(
                    bus,
                    sdbusRule::interfacesAdded() +
                    sdbusRule::argNpath(0, path),
                    std::bind(std::mem_fn(&Manager::cpuCreated),
                        this, std::placeholders::_1));
        }
    }
    else
    {
        for (const auto& occ : occs)
        {
            // CPU inventory exists already, OCC objects can be created.
            createObjects(occ);
        }
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
            std::make_unique<PassThrough>(
                bus,
                path.c_str()));

    statusObjects.emplace_back(
            std::make_unique<Status>(
                bus,
                event,
                path.c_str(),
                std::bind(std::mem_fn(&Manager::statusCallBack),
                    this, std::placeholders::_1)));

    // Create the power cap monitor object for master occ (0)
    if (!pcap)
    {
        pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
                bus,
                *statusObjects.front());
    }

    // Now that the objects are created, send a HeartBeat command to
    // Host and if that responds with Success, then set the OCC Active
    // status to "true"
    sendHeartBeat();
}

void Manager::statusCallBack(bool status)
{
    using namespace phosphor::logging;
    using InternalFailure = sdbusplus::xyz::openbmc_project::Common::
        Error::InternalFailure;

    // At this time, it won't happen but keeping it
    // here just in case something changes in the future
    if ((activeCount == 0) && (!status))
    {
        log<level::ERR>("Invalid update on OCCActive");
        elog<InternalFailure>();
    }

    activeCount += status ? 1 : -1;

    // If all the OCCs are bound, then start error detection
    if (activeCount == statusObjects.size())
    {
        for (const auto& occ: statusObjects)
        {
            occ->addErrorWatch();
        }
    }
    else if (!status)
    {
        // If some OCCs are not bound yet, those will be a NO-OP
        for (const auto& occ: statusObjects)
        {
            occ->removeErrorWatch();
        }
    }
}

// Sends heartbeat message to host control command handler
void Manager::sendHeartBeat()
{
    using namespace phosphor::logging;
    constexpr auto CONTROL_HOST_PATH = "/xyz/openbmc_project/control/host0";
    constexpr auto CONTROL_HOST_INTF = "xyz.openbmc_project.Control.Host";

    // This will throw exception on failure
    auto service = getService(bus, CONTROL_HOST_PATH, CONTROL_HOST_INTF);

    auto method = bus.new_method_call(service.c_str(),
                                      CONTROL_HOST_PATH,
                                      CONTROL_HOST_INTF,
                                      "Execute");
    // Heartbeat control command
    method.append(convertForMessage(
                      HostControl::Host::Command::Heartbeat).c_str());

    bus.call_noreply(method);
    return;
}

// Handler called by Host control command handler to convey the
// status of the executed command
void Manager::hostControlEvent(sdbusplus::message::message& msg)
{
    using namespace phosphor::logging;

    std::string cmdCompleted{};
    std::string cmdStatus{};

    msg.read(cmdCompleted, cmdStatus);

    if(HostControl::Host::convertCommandFromString(cmdCompleted) ==
            HostControl::Host::Command::Heartbeat)
    {
        if(HostControl::Host::convertResultFromString(cmdStatus) ==
                HostControl::Host::Result::Success)
        {
            // Host responded to Heartbeat. Set the OccActive to true
            for (const auto& status: statusObjects)
            {
                status->occActive(true);
            }
        }
        else
        {
            // Must be a Timeout. Log an error trace
            log<level::ERR>("Error invoking Heartbeat command.");
        }
    }

    // Not watching for any other command now. Add when something
    // gets watched later
    return;
}

#ifdef I2C_OCC
void Manager::initStatusObjects()
{
    // Make sure we have a valid path string
    static_assert(sizeof(DEV_PATH) != 0);

    auto deviceNames = i2c_occ::getOccHwmonDevices(DEV_PATH);
    for (auto& name : deviceNames)
    {
        i2c_occ::i2cToDbus(name);
        auto path = fs::path(OCC_CONTROL_ROOT) / name;
        statusObjects.emplace_back(
                std::make_unique<Status>(
                    bus,
                    event,
                    path.c_str()));
    }
}
#endif

} // namespace occ
} // namespace open_power
