#pragma once

#include "occ_command.hpp"

#include <fmt/core.h>

#include <org/open_power/OCC/PassThrough/server.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <string>

namespace open_power
{
namespace occ
{

using Iface = sdbusplus::server::object::object<
    sdbusplus::org::open_power::OCC::server::PassThrough>;

// For waiting on signals
namespace sdbusRule = sdbusplus::bus::match::rules;

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
    PassThrough(sdbusplus::bus::bus& bus, const char* path);

    /** @brief Pass through command to OCC from dbus
     *  @param[in] command - command to pass-through
     *  @returns OCC response as an array
     */
    std::vector<std::int32_t> send(std::vector<std::int32_t> command) override;

    /** @brief Pass through command to OCC from openpower-occ-control
     *  @param[in] command - command to pass-through
     *  @returns OCC response as an array
     */
    std::vector<std::uint8_t> send(std::vector<std::uint8_t> command);

  private:
    /** @brief Pass-through occ path on the bus */
    std::string path;

    /** @brief OCC device path
     *  For now, here is the hard-coded mapping until
     *  the udev rule is in.
     *  occ0 --> /dev/occ1
     *  occ1 --> /dev/occ2
     *  ...
     */
    std::string devicePath;

    /** @brief OCC instance number */
    int occInstance;

    /** @brief Indicates whether or not the OCC is currently active */
    bool occActive = false;

    /** @brief Subscribe to OCC Status signal
     *
     *  Once the OCC status gets to active, only then we will get /dev/occ2
     *  populated and hence need to wait on that before opening that
     */
    sdbusplus::bus::match_t activeStatusSignal;

    /** @brief Object to send commands to the OCC */
    OccCommand occCmd;

    /** @brief Callback function on OCC Status change signals
     *
     *  @param[in]  msg - Data associated with subscribed signal
     */
    void activeStatusEvent(sdbusplus::message::message& msg);
};

} // namespace occ
} // namespace open_power
