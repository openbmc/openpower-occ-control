#pragma once

#include <xyz/openbmc_project/Association/Definitions/server.hpp>
#include <xyz/openbmc_project/Sensor/Value/server.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/server.hpp>

namespace open_power
{
namespace occ
{
namespace dbus
{

using ObjectPath = std::string;

using SensorIntf = sdbusplus::server::object_t<
    sdbusplus::xyz::openbmc_project::Sensor::server::Value>;
using OperationalStatusIntf =
    sdbusplus::server::object_t<sdbusplus::xyz::openbmc_project::State::
                                    Decorator::server::OperationalStatus>;

// Note: Not using object<> so the PropertiesVariant ctor is available.
using AssociationIntf =
    sdbusplus::xyz::openbmc_project::Association::server::Definitions;

/** @class OccDBusSensors
 *  @brief This is a custom D-Bus object, used to add D-Bus interface and update
 *         the corresponding properties value.
 */
class OccDBusSensors
{
  private:
    OccDBusSensors() {}

  public:
    OccDBusSensors(const OccDBusSensors&) = delete;
    OccDBusSensors(OccDBusSensors&&) = delete;
    OccDBusSensors& operator=(const OccDBusSensors&) = delete;
    OccDBusSensors& operator=(OccDBusSensors&&) = delete;
    ~OccDBusSensors() = default;

    static OccDBusSensors& getOccDBus()
    {
        static OccDBusSensors customDBus;
        return customDBus;
    }

  public:
    /** @brief Set the max value of the Sensor
     *
     *  @param[in] path  - The object path
     *  @param[in] value - The value of the MaxValue property
     *
     *  @return true or false
     */
    bool setMaxValue(const std::string& path, double value);

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
     *  @param[in] value - The value of the MinValue property
     *
     *  @return true or false
     */
    bool setMinValue(const std::string& path, double value);

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
     *  @param[in] value - The value of the Value property
     *
     *  @return true or false
     */
    bool setValue(const std::string& path, double value);

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
     *  @param[in] value - The value of the Unit property
     *
     *  @return true or false
     */
    bool setUnit(const std::string& path, const std::string& value);

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
     *  @param[in] value  - PLDM operational fault status
     *
     *  @return true or false
     */
    bool setOperationalStatus(const std::string& path, bool value);

    /** @brief Get the Functional property
     *
     *  @param[in] path   - The object path
     *
     *  @return status    - PLDM operational fault status
     */
    bool getOperationalStatus(const std::string& path) const;

    /** @brief Returns the Chassis inventory path
     *
     * @return path       - The chassis D-Bus path
     */
    std::string getChassisPath();

    /** @brief Set the association to the chassis
     *
     *  @param[in] path        - The object path
     *  @param[in] fType       - vector of forward types
     */
    void setChassisAssociation(const std::string& path,
                               const std::vector<std::string>& fTypes);

    /** @brief Set the value of the DVFS temp sensor
     *
     *  @param[in] path  - The object path
     *  @param[in] value - The value of the Value property
     */
    void setDvfsTemp(const std::string& path, double value);

    /** @brief Says if the DVFS temp sensor is already present
     *
     *  @param[in] value - The value of the Value property
     *  @return bool - If the sensor is already present
     */
    bool hasDvfsTemp(const std::string& path) const;

  private:
    std::map<ObjectPath, std::unique_ptr<SensorIntf>> sensors;

    std::map<ObjectPath, std::unique_ptr<OperationalStatusIntf>>
        operationalStatus;

    std::map<ObjectPath, std::unique_ptr<AssociationIntf>> chassisAssociations;

    std::string chassisPath;

    /** @brief Map of DVFS (Dynamic Voltage and Frequency Slewing) temps
     *
     * These do not have associations and do not get set to NaN when the OCC
     * isn't active.
     */
    std::map<std::string, std::unique_ptr<SensorIntf>> dvfsTemps;
};

} // namespace dbus
} // namespace occ
} // namespace open_power
