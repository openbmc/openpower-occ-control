#pragma once

#include <string>
#include <vector>
#include <unistd.h>
#include <sdbusplus/bus.hpp>
#include <functional>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/PassThrough/server.hpp>
#include "config.h"
#include "file.hpp"

namespace sdbusRule = sdbusplus::bus::match::rules;

namespace open_power
{
namespace occ
{
namespace pass_through
{

class PassThrough;

namespace manager
{

static constexpr auto cpu0Path =
    "/xyz/openbmc_project/inventory/system/chassis/motherboard/cpu0";
static constexpr auto cpu1Path =
    "/xyz/openbmc_project/inventory/system/chassis/motherboard/cpu1";

/** @class Manager
 *  @brief Builds and manages OCC pass-through objects
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
            cpuMatches.emplace_back(
                bus,
                sdbusRule::type::signal() +
                sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(cpu0Path) +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
                std::bind(std::mem_fn(&Manager::cpu0Created),
                          this, std::placeholders::_1));

            cpuMatches.emplace_back(
                bus,
                sdbusRule::type::signal() +
                sdbusRule::member("PropertiesChanged") +
                sdbusRule::path(cpu1Path) +
                sdbusRule::interface("org.freedesktop.DBus.Properties"),
                std::bind(std::mem_fn(&Manager::cpu1Created),
                          this, std::placeholders::_1));
        }

        /** @brief Callback that responds to cpu0 creation in the inventory -
         *         by creating the occ0 passthrough object.
         *
         *  @param[in] msg - bus message
         *
         *  @returns int - 1 to indicate the callback is single shot
         */
        int cpu0Created(sdbusplus::message::message& msg)
        {
            objects.emplace_back(
                std::make_unique<PassThrough>(
                    bus,
                    (std::string(OCC_PASS_THROUGH_ROOT) + "/occ0").c_str()));

            // Return 1 so that the callback is single shot
            return 1;
        }

        /** @brief Callback that responds to cpu1 creation in the inventory -
         *         by creating the occ1 passthrough object.
         *
         *  @param[in] msg - bus message
         *
         *  @returns int - 1 to indicate the callback is single shot
         */
        int cpu1Created(sdbusplus::message::message& msg)
        {
            objects.emplace_back(
                std::make_unique<PassThrough>(
                    bus,
                    (std::string(OCC_PASS_THROUGH_ROOT) + "/occ1").c_str()));

            // Return 1 so that the callback is single shot
            return 1;
        }

    private:
        /** @brief reference to the bus */
        sdbusplus::bus::bus& bus;

        /** @brief OCC pass-through objects */
        std::vector<std::unique_ptr<PassThrough>> objects;

        /** @brief sbdbusplus match objects */
        std::vector<sdbusplus::bus::match_t> cpuMatches;
};

} // namespace manager


using Iface = sdbusplus::server::object::object<
    sdbusplus::org::open_power::OCC::server::PassThrough>;

/** @class PassThrough
 *  @brief Implements org.open_power.OCC.PassThrough
 */
class PassThrough : public Iface
{
    public:
        PassThrough() = delete;
        ~PassThrough() = default;
        PassThrough(const PassThrough&) = delete;
        PassThrough& operator=(const PassThrough&) = delete;
        PassThrough(PassThrough&&) = default;
        PassThrough& operator=(PassThrough&&) = default;

        /** @brief Ctor to put pass-through d-bus object on the bus
         *  @param[in] bus - Bus to attach to
         *  @param[in] path - Path to attach at
         */
        PassThrough(sdbusplus::bus::bus& bus,
                    const char* path);

        /** @brief Pass through command to OCC
         *  @param[in] command - command to pass-through
         *  @returns OCC response as an array
         */
        std::vector<std::int32_t>
            send(std::vector<std::int32_t> command) override;

    private:
        /** @brief Pass-through occ path on the bus */
        std::string path;

        /** @brief OCC device path
         *  For now, here is the hard-coded mapping until
         *  the udev rule is in.
         *  occ0 --> /dev/occfifo1
         *  occ1 --> /dev/occfifo2
         *  ...
         */
        std::string devicePath = "/dev/occfifo";

        /** @brief File descriptor manager */
        FileDescriptor fd;

        /** Opens devicePath and returns file descritor */
        int openDevice();
};

} // namespace pass_through
} // namespace occ
} // namespace open_power
