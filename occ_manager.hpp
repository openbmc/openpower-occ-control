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
#include "i2c_occ.hpp"

namespace sdbusRule = sdbusplus::bus::match::rules;
namespace open_power
{
namespace occ
{
constexpr auto invRootPath  = "/xyz/openbmc_project/inventory";
constexpr auto cpuIntf = "xyz.openbmc_project.Inventory.Item.Cpu";
using DbusValue = sdbusplus::message::variant<bool, int64_t, std::string>;
using DbusProperty = std::string;
using DbusPropertyMap = std::map<DbusProperty, DbusValue>;
using DbusInterface = std::string;
using DbusInterfaceMap = std::map<DbusInterface, DbusPropertyMap>;

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
            auto occs = open_power::occ::finder::get(bus);
            if (occs.empty())
            {
                cpuMatch = std::make_unique<sdbusplus::bus::match_t>(
                                bus,
                                sdbusRule::interfacesAdded() +
                                sdbusRule::path_namespace(invRootPath),
                                std::bind(std::mem_fn(&Manager::cpuCreated),
                                this, std::placeholders::_1));
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
            //Interface data is of the fromat a{sa{sv}}
            namespace fs = std::experimental::filesystem;

            sdbusplus::message::object_path objectPath;
            msg.read(objectPath);

            DbusInterfaceMap intfList;
            msg.read(intfList);
            const auto& it = intfList.find(cpuIntf);
            if (it != intfList.end())
            {
                fs::path cpuPath((std::string(std::move(objectPath))));
                auto name = cpuPath.filename().string();
                auto index = name.find(CPU_NAME);
                name.replace(index, std::strlen(CPU_NAME), OCC_NAME);
                createObjects(name);
            }
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
        std::unique_ptr<sdbusplus::bus::match_t> cpuMatch;

        /** @brief Number of OCCs that are bound */
        uint8_t activeCount = 0;

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
};

} // namespace occ
} // namespace open_power
