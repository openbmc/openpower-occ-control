#pragma once

#include <cstring>
#include <vector>
#include <experimental/filesystem>
#include <functional>
#include <sdbusplus/bus.hpp>
#include "occ_pass_through.hpp"
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

        /** @brief Ctor - Add OCC pass-through objects on the bus. Create
         *         OCC objects when corresponding CPU inventory is created.
         *  @param[in] bus - handle to the bus
         */
        Manager(sdbusplus::bus::bus& bus):
            bus(bus)
        {
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

            auto occPath = fs::path(OCC_PASS_THROUGH_ROOT);
            std::string name{cpu.c_str()};
            auto index = name.find(CPU_NAME);
            name.replace(index, std::strlen(CPU_NAME), OCC_NAME);
            occPath /= name;

            objects.emplace_back(
                std::make_unique<PassThrough>(
                    bus,
                    occPath.c_str()));

            return 0;
        }

    private:
        /** @brief reference to the bus */
        sdbusplus::bus::bus& bus;

        /** @brief OCC pass-through objects */
        std::vector<std::unique_ptr<PassThrough>> objects;

        /** @brief sbdbusplus match objects */
        std::vector<sdbusplus::bus::match_t> cpuMatches;
};

} // namespace occ
} // namespace open_power
