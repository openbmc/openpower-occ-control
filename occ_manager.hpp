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
#include "utils.hpp"

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
         *
         *  @param[in] bus   - handle to the bus
         *  @param[in] event - Unique ptr reference to sd_event
         */
        Manager(sdbusplus::bus::bus& bus,
                EventPtr& event) :
            bus(bus),
            event(event)
        {
#ifdef I2C_OCC
            // I2C OCC status objects are initialized directly
            initStatusObjects();
#else
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
#endif
        }

        /** @brief Callback that responds to cpu creation in the inventory -
         *         by creating the needed objects.
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

            auto name = cpuPath.filename().string();
            auto index = name.find(CPU_NAME);
            name.replace(index, std::strlen(CPU_NAME), OCC_NAME);

            createObjects(name);

            return 0;
        }

    private:
        /** @brief Create child OCC objects.
         *
         *  @param[in] occ - the occ name, such as occ0.
         */
        void createObjects(const std::string& occ)
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
                    path.c_str()));

            // Create the power cap monitor object for master occ (0)
            if (!pcap)
            {
                pcap = std::make_unique<open_power::occ::powercap::PowerCap>(
                                                        bus,
                                                        *statusObjects.front());
            }
        }

        /** @brief reference to the bus */
        sdbusplus::bus::bus& bus;

        /** @brief reference to sd_event wrapped in unique_ptr */
        EventPtr& event;

        /** @brief OCC pass-through objects */
        std::vector<std::unique_ptr<PassThrough>> passThroughObjects;

        /** @brief OCC Status objects */
        std::vector<std::unique_ptr<Status>> statusObjects;

        /** @brief Power cap monitor and occ notification object */
        std::unique_ptr<open_power::occ::powercap::PowerCap> pcap;

        /** @brief sbdbusplus match objects */
        std::vector<sdbusplus::bus::match_t> cpuMatches;

#ifdef I2C_OCC
        /** @brief Init Status objects for I2C OCC devices
         *
         * It iterates in /sys/bus/i2c/devices, finds all occ hwmon devices
         * and creates status objects.
         */
        void initStatusObjects()
        {
            // Make sure we have a valid path string
            static_assert(sizeof(DEV_PATH) != 0);

            auto deviceNames = utils::getOccHwmonDevices(DEV_PATH);
            for (auto& name : deviceNames)
            {
                utils::i2cToDbus(name);
                auto path = fs::path(OCC_CONTROL_ROOT) / name;
                statusObjects.emplace_back(
                    std::make_unique<Status>(
                        bus,
                        event,
                        path.c_str()));
            }
        }
#endif
};

} // namespace occ
} // namespace open_power
