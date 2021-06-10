#pragma once

#include "utils.hpp"

#include <map>
#include <memory>
#include <sdbusplus/server.hpp>
#include <string>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/server.hpp>

namespace open_power
{
namespace occ
{
namespace dbus
{

using namespace open_power::occ::utils;
using ObjectPath = std::string;

using SensorIntf = sdbusplus::xyz::openbmc_project::Sensor::server::Value;
using OperationalStatusIntf = sdbusplus::xyz::openbmc_project::State::
    Decorator::server::OperationalStatus;

/** @class OccDBus
 *  @brief This is a custom D-Bus object, used to add D-Bus interface and update
 *         the corresponding properties value.
 */
class OccDBus
{
  private:
    OccDBus()
    {
    }

  public:
    OccDBus(const OccDBus&) = delete;
    OccDBus(OccDBus&&) = delete;
    OccDBus& operator=(const OccDBus&) = delete;
    OccDBus& operator=(OccDBus&&) = delete;
    ~OccDBus() = default;

    static OccDBus& getOccDBus()
    {
        static OccDBus customDBus;
        return customDBus;
    }

  public:
    /** @brief Set the max value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @param[in] value - The value of the MaxValue property
     */
    void setMaxValue(const std::string& path, double value);

    /** @brief Get the max value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @return bool     - The value of the MaxValue property
     */
    double getMaxValue(const std::string& path) const;

    /** @brief Set the min value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @param[in] value - The value of the MinValue property
     */
    void setMinValue(const std::string& path, double value);

    /** @brief Get the min value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @return bool     - The value of the MinValue property
     */
    double getMinValue(const std::string& path) const;

    /** @brief Set the value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @param[in] value - The value of the Value property
     */
    void setValue(const std::string& path, double value);

    /** @brief Get the value of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @return bool     - The value of the Value property
     */
    double getValue(const std::string& path) const;

    /** @brief Set the unit of the Sensor
     *
     *  @param[in] path  - The object path
     *
     *  @param[in] value - The value of the Unit property
     */
    void setUnit(const std::string& path, const std::string& value);

    /** @brief Get the unit of the Sensor
     *
     *  @param[in] path       - The object path
     *
     *  @return std::string   - The value of the Unit property
     */
    std::string getUnit(const std::string& path) const;

    /** @brief Set the Functional property
     *
     *  @param[in] path   - The object path
     *
     *  @param[in] value  - PLDM operational fault status
     */
    void setOperationalStatus(const std::string& path, bool value);

    /** @brief Get the Functional property
     *
     *  @param[in] path   - The object path
     *
     *  @return status    - PLDM operational fault status
     */
    bool getOperationalStatus(const std::string& path) const;

  private:
    std::map<ObjectPath, std::unique_ptr<SensorIntf>> sensors;

    std::map<ObjectPath, std::unique_ptr<OperationalStatusIntf>>
        operationalStatus;
};

} // namespace dbus
} // namespace occ
} // namespace open_power
