#pragma once

#include "occ_errors.hpp"
#include "utils.hpp"

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

enum class CmdType
{
    POLL = 0x00,
    CLEAR_ERROR_LOG = 0x12,
    SET_MODE_AND_STATE = 0x20,
    SET_CONFIG_DATA = 0x21,
    SET_USER_PCAP = 0x22,
    RESET_PREP = 0x25,
    SEND_AMBIENT = 0x30,
    DEBUG_PASS_THROUGH = 0x40,
    AME_PASS_THROUGH = 0x41,
    GET_FIELD_DEBUG_DATA = 0x42,
    MFG_TEST = 0x53
};

enum class OccState
{
    NO_CHANGE = 0x00,
    STANDBY = 0x01,
    OBSERVATION = 0x02,
    ACTIVE = 0x03,
    SAFE = 0x04,
    CHARACTERIZATION = 0x05,
};

enum class SysPwrMode
{
    NO_CHANGE = 0,
    DISABLE = 0x01,      // Disable / Static Base Frequencey
    SFP = 0x03,          // Static Frequency Point (requires freqPt)
    SAFE = 0x04,         // reported when system is in SAFE mode (not settable)
    POWER_SAVING = 0x05, // Static Power Saving
    DYNAMIC_PERF = 0x0A, // Dynamic / Balanced Performance
    FFO = 0x0B,          // Fixed Frequency Override (requires freqPt)
    MAX_PERF = 0x0C      // Maximum Performance
};

// Only some of the SysPwrModes are currently supported and allowed to be set
#define VALID_POWER_MODE_SETTING(mode)                                         \
    ((mode == SysPwrMode::DISABLE) || (mode == SysPwrMode::POWER_SAVING) ||    \
     (mode == SysPwrMode::DYNAMIC_PERF) || (mode == SysPwrMode::MAX_PERF))

enum class RspStatus
{
    SUCCESS = 0x00,
    CONDITIONAL_SUCCESS = 0x01,
    INVALID_COMMAND = 0x11,
    INVALID_COMMAND_LENGTH = 0x12,
    INVALID_DATA_FIELD = 0x13,
    CHECKSUM_FAILURE = 0x14,
    INTERNAL_ERROR = 0x15,
    PRESENT_STATE_PROHIBITS = 0x16,
    COMMAND_IN_PROGRESS = 0xFF
};

enum class CmdStatus
{
    SUCCESS,
    OPEN_FAILURE,
    FAILURE,
    INVALID_CHECKSUM
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
     *  @param[in] path - Path to attach at
     */
    OccCommand(uint8_t instance, const char* path);

    /** @brief Dtor to clean up and close device */
    ~OccCommand()
    {
        closeDevice();
    }

    /** @brief Send the command to the OCC and collect the response.
     * The checksum will be validated and removed from the response.
     *
     *  @param[in] command - command to pass-through
     *  @param[out] response - response
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

    /** Opens devicePath and populates file descriptor */
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
