#pragma once

#include <string>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <org/open_power/OCC/PassThrough/server.hpp>
#include "file.hpp"

namespace open_power
{
namespace occ
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
        std::string devicePath = "/dev/occ";

        /** @brief File descriptor manager */
        FileDescriptor fd;

        /** Opens devicePath and returns file descritor */
        int openDevice();
};

} // namespace occ
} // namespace open_power
