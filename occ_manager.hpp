#pragma once

#include <cstring>
#include <vector>
#include <functional>
#include <sdbusplus/bus.hpp>
#include "occ_pass_through.hpp"
#include "occ_status.hpp"
#include "powercap.hpp"

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
            findAndCreateObjects();
#endif
        }

        inline int getNumOCCs() const
        {
            return activeCount;
        }

    private:
        /** @brief Checks if the CPU inventory is present and if so, creates
         *         the occ D-Bus objects. Else, registers a handler to be
         *         called when inventory is created.
         */
        void findAndCreateObjects();

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
         *
         *  @param[in] status - OccActive status
         */
        void statusCallBack(bool status);

        /** @brief Sends a Heartbeat command to host control command handler */
        void sendHeartBeat();

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

#ifdef I2C_OCC
        /** @brief Init Status objects for I2C OCC devices
         *
         * It iterates in /sys/bus/i2c/devices, finds all occ hwmon devices
         * and creates status objects.
         */
        void initStatusObjects();
#endif
};

} // namespace occ
} // namespace open_power
