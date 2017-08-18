#include <experimental/filesystem>
#include <functional>
#include "occ_manager.hpp"
#include "occ_finder.hpp"
#include "config.h"
namespace open_power
{
namespace occ
{

namespace sdbusRule = sdbusplus::bus::match::rules;

Manager::Manager(sdbusplus::bus::bus& bus, EventPtr& event) :
        bus(bus),
        event(event)
{
    // Check if CPU inventory exists already.
    auto occs = open_power::occ::finder::get();
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
}

void Manager::statusCallBack(bool value)
{
    activeCount += value ? 1 : -1;

    // If all the OCCs are bound, then start error detection
    if (activeCount == statusObjects.size())
    {
        for (const auto& occ: statusObjects)
        {
            occ->addErrorWatch();
        }
    }
    else if (!value)
    {
        // If some OCCs are not bound yet, those will be a NO-OP
        for (const auto& occ: statusObjects)
        {
            occ->removeErrorWatch();
        }
    }
}

} // namespace occ
} // namespace open_power
