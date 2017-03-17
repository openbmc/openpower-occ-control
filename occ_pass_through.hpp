#pragma once

#include <string>
#include <vector>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include "org/open_power/OCC/PassThrough/server.hpp"

namespace open_power
{
namespace occ
{
namespace pass_through
{

using Iface = sdbusplus::server::object::object<
    sdbusplus::org::open_power::OCC::server::PassThrough>;

/** @class PassThrough
 *  @brief Implements org.open_power.OCC.PassThrough
 */
class PassThrough : public Iface
{
    public:
        PassThrough() = delete;
        PassThrough(const PassThrough&) = delete;
        PassThrough& operator=(const PassThrough&) = delete;
        PassThrough(PassThrough&&) = default;
        PassThrough& operator=(PassThrough&&) = default;
        ~PassThrough() = default;

        /** @brief Ctor to put pass-through d-bus object on the bus
         *  @param[in] bus - Bus to attach to
         *  @param[in] path - Path to attach at
         */
        PassThrough(sdbusplus::bus::bus& bus,
                    const char* path);

        /** @brief Pass through command to OCC
         *  @param[in] command - command to pass-through
         *  @returns OCC response as an array of strings
         */
        std::vector<std::string>
            send(std::vector<std::string> command) override;

    private:
        /** @brief Pass-through occ path on the bus */
        std::string path;
};

} // namespace pass_through
} // namespace occ
} // namespace open_power
