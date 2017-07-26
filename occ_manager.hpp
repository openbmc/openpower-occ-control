#pragma once

#include <cstring>
#include <vector>
#include <experimental/filesystem>
#include <functional>
#include <sdbusplus/bus.hpp>
#include <powercap.hpp>
#include "occ_pass_through.hpp"
#include "occ_status.hpp"
#include "occ_finder.hpp"
#include "config.h"

namespace sdbusRule = sdbusplus::bus::match::rules;

namespace open_power
{
namespace occ
{

/** @class Manager
 *  @brief Builds and manages OCC objects
 */
struct Manager
{
    public:
        Manager() = delete;
        Manager(const Manager&) = delete;
        Manager& operator=(const Manager&) = delete;
        Manager(Manager&&) = default;
        Manager& operator=(Manager&&) = default;
        ~Manager() = default;

        /** @brief Adds OCC pass-through and status objects on the bus
         *         when corresponding CPU inventory is created.
         *  @param[in] bus - handle to the bus
         */
        Manager(sdbusplus::bus::bus& bus):
            bus(bus)
        {
            // Check if CPU inventory exists already, if yes the OCC objects
            // can be created right away.
            auto occs = open_power::occ::finder::get();
            for (const auto& occ : occs)
            {
                auto occPath = fs::path(OCC_CONTROL_ROOT);
                occPath /= occ;
                passThroughObjects.emplace_back(
                    std::make_unique<PassThrough>(
                        bus,
                        occPath.c_str()));
                statusObjects.emplace_back(
                    std::make_unique<Status>(
                        bus,
                        occPath.c_str()));
            }

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
        }

        /** @brief Callback that responds to cpu creation in the inventory -
         *         by creating the occ passthrough and status objects.
         *
         *  @param[in] msg - bus message
         *
         *  @returns 0 to indicate success
         */
        int cpuCreated(sdbusplus::message::message& msg)
        {
            namespace fs = std::experimental::filesystem;

            sdbusplus::message::object_path o;
            msg.read(o);
            fs::path cpuPath(std::string(std::move(o)));
            auto cpu = cpuPath.filename();

            std::string name{cpu.c_str()};
            auto index = name.find(CPU_NAME);
            name.replace(index, std::strlen(CPU_NAME), OCC_NAME);

            auto path = fs::path(OCC_CONTROL_ROOT) / name;
            passThroughObjects.emplace_back(
                std::make_unique<PassThrough>(
                    bus,
                    path.c_str()));

            statusObjects.emplace_back(
                std::make_unique<Status>(
                    bus,
                    path.c_str()));

            // Create the power cap monitor object for master occ (0)
            if(!pcap && (index == 0))
            {
                pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
                                                        bus,
                                                        *statusObjects[index]);
            }
            return 0;
        }

    private:
        /** @brief reference to the bus */
        sdbusplus::bus::bus& bus;

        /** @brief OCC pass-through objects */
        std::vector<std::unique_ptr<PassThrough>> passThroughObjects;

        /** @brief OCC Status objects */
        std::vector<std::unique_ptr<Status>> statusObjects;

        /** @brief Power cap monitor and occ notification object */
        std::unique_ptr<open_power::occ::powercap::PowerCap> pcap;

        /** @brief sbdbusplus match objects */
        std::vector<sdbusplus::bus::match_t> cpuMatches;
};

} // namespace occ
} // namespace open_power
