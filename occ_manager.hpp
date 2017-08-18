#pragma once

#include <cstring>
#include <vector>
#include <sdbusplus/bus.hpp>
#include <powercap.hpp>
#include "occ_pass_through.hpp"
#include "occ_status.hpp"
#include "occ_events.hpp"
#include "config.h"
namespace open_power
{
namespace occ
{

/** @class Manager
 *  @brief Builds and manages OCC objects
 */
class Manager
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
        Manager(sdbusplus::bus::bus& bus, EventPtr& event);

    private:
        /** @brief Callback that responds to cpu creation in the inventory -
         *         by creating the needed objects.
         *
         *  @param[in] msg - bus message
         *
         *  @returns 0 to indicate success
         */
        int cpuCreated(sdbusplus::message::message& msg);

        /** @brief Create child OCC objects.
         *
         *  @param[in] occ - the occ name, such as occ0.
         */
        void createObjects(const std::string& occ);

        /** @brief Callback handler invoked by Status object when the OccActive
         *         property is changed. This is needed to make sure that the
         *         error detection is started only after all the OCCs are bound.
         *         Similarly, when one of the OCC gets its OccActive property
         *         un-set, then the OCC error detection needs to be stopped on
         *         all the OCCs
         */
        void statusCallBack(bool status);

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
