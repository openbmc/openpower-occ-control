#pragma once

#include "occ_command.hpp"
#include "powermode.hpp"
#include "utils.hpp"

#include <org/open_power/OCC/PassThrough/server.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>

#include <format>
#include <string>

namespace open_power
{
namespace occ
{

using Iface = sdbusplus::server::object_t<
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
     *  @param[in] path - Path to attach at
     */
    explicit PassThrough(
        const char* path,
        std::unique_ptr<open_power::occ::powermode::PowerMode>& powerModeRef);

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

    /** @brief Set a Power Mode
     *
     *  @param[in] mode - desired System Power Mode
     *  @param[in] modeData - data associated some Power Modes
     *
     *  @returns true if mode change was accepted
     */
    bool setMode(const uint8_t mode, const uint16_t modeData);

  private:
    /** @brief Pass-through occ path on the bus */
    std::string path;

    /** @brief OCC PowerMode object */
    std::unique_ptr<open_power::occ::powermode::PowerMode>& pmode;

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
    void activeStatusEvent(sdbusplus::message_t& msg);
};

} // namespace occ
} // namespace open_power
