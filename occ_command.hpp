#pragma once

#include "occ_errors.hpp"

#include <org/open_power/OCC/PassThrough/server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/server/object.hpp>
#include <string>

namespace open_power
{
namespace occ
{

// For waiting on signals
namespace sdbusRule = sdbusplus::bus::match::rules;

enum class CmdStatus
{
    SUCCESS,
    OPEN_FAILURE,
    FAILURE
};

/** @brief Trace block of data in hex with log<level:INFO>
 *
 *  @param[in] data - vector containing data to trace
 *  @param[in] data_len - optional number of bytes to trace
 *  If 0, entire vector will be traced.
 */
void dump_hex(const std::vector<std::uint8_t>& data,
              const unsigned int data_len = 0);

/** @class OccCommand
 *  @brief Send commands and process respsonses from the OCC
 */
class OccCommand
{
  public:
    OccCommand() = delete;
    OccCommand(const OccCommand&) = delete;
    OccCommand& operator=(const OccCommand&) = delete;
    OccCommand(OccCommand&&) = default;
    OccCommand& operator=(OccCommand&&) = default;

    /** @brief Ctor to set up which OCC the command will go to
     *
     *  @param[in] instance - OCC instance
     *  @param[in] bus - Bus to attach to
     *  @param[in] path - Path to attach at
     */
    OccCommand(uint8_t instance, sdbusplus::bus::bus& bus, const char* path);

    /** @brief Dtor to clean up and close device */
    ~OccCommand()
    {
        closeDevice();
    }

    /** @brief Send the command to the OCC and collect the response
     *
     *  @param[in] command - command to pass-through
     *  @param[out] rsponse - response
     *  returns SUCCESS if response was received
     */
    CmdStatus send(const std::vector<std::uint8_t>& command,
                   std::vector<std::uint8_t>& response);

  private:
    /** @brief Instance number of the target OCC */
    uint8_t occInstance;

    /** @brief OCC path on the bus */
    std::string path;

    /** @brief OCC device path
     *  For now, here is the hard-coded mapping until
     *  the udev rule is in.
     *  occ0 --> /dev/occ1
     *  occ1 --> /dev/occ2
     *  ...
     */
    std::string devicePath;

    /** @brief Indicates whether or not the OCC is currently active */
    bool occActive = false;

    /** brief file descriptor associated with occ device */
    int fd = -1;

    /** @brief Subscribe to OCC Status signal
     *
     *  Once the OCC status gets to active, only then we will get /dev/occ2
     *  populated and hence need to wait on that before opening that
     */
    sdbusplus::bus::match_t activeStatusSignal;

    /** Opens devicePath and populates file descritor */
    void openDevice();

    /** Closed the fd associated with opened device */
    void closeDevice();

    /** @brief Callback function on OCC Status change signals
     *
     *  @param[in]  msg - Data associated with subscribed signal
     */
    void activeStatusEvent(sdbusplus::message::message& msg);
};

} // namespace occ
} // namespace open_power
