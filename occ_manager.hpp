#pragma once

#include <cstring>
#include <vector>
#include <experimental/filesystem>
#include <functional>
#include <sdbusplus/bus.hpp>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog.hpp>
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
         *
         *  @param[in] bus   - handle to the bus
         *  @param[in] event - Unique ptr reference to sd_event
         */
        Manager(sdbusplus::bus::bus& bus,
                EventPtr& event) :
            bus(bus),
            event(event)
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

    private:
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

        /** @brief Callback handler invoked by Status object when the OccActive
         *         property is changed. This is needed to make sure that the
         *         error detection is started only after all the OCCs are bound.
         *         Similarly, when one of the OCC gets its OccActive property
         *         un-set, then the OCC error detection needs to be stopped on
         *         all the OCCs
         *
         *  @param[in] status - OccActive status
         */
        void statusCallBack(bool status)
        {
            using namespace phosphor::logging;
            using InternalFailure = sdbusplus::xyz::openbmc_project::Common::
                                        Error::InternalFailure;

            // At this time, it won't happen but keeping it
            // here just incase something changes in the future
            if (activeCount == 0 && !status)
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

        /** @brief Number of OCCs that are bound */
        uint8_t activeCount = 0;
};

} // namespace occ
} // namespace open_power
