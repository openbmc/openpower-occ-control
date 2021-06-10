#include "occ_dbus.hpp"

#include "utils.hpp"

#include <iostream>
#include <phosphor-logging/log.hpp>

namespace open_power
{
namespace occ
{
namespace dbus
{

using namespace phosphor::logging;
void OccDBusSensors::setMaxValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->maxValue(value);
}

double OccDBusSensors::getMaxValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->maxValue();
    }

    throw std::invalid_argument("Failed to get MaxValue property.");
}

void OccDBusSensors::setMinValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->minValue(value);
}

double OccDBusSensors::getMinValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->minValue();
    }

    throw std::invalid_argument("Failed to get MinValue property.");
}

void OccDBusSensors::setValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->value(value);
}

double OccDBusSensors::getValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->value();
    }

    throw std::invalid_argument("Failed to get Value property.");
}

void OccDBusSensors::setUnit(const std::string& path, const std::string& value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    try
    {
        sensors.at(path)->unit(SensorIntf::convertUnitFromString(value));
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("set Unit propety failed", entry("ERROR=%s", e.what()));
    }
}

std::string OccDBusSensors::getUnit(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        try
        {
            return SensorIntf::convertUnitToString(sensors.at(path)->unit());
        }
        catch (const std::exception& e)
        {
            log<level::ERR>("get Unit propety failed",
                            entry("ERROR=%s", e.what()));
        }
    }

    throw std::invalid_argument("Failed to get Unit property.");
}

void OccDBusSensors::setOperationalStatus(const std::string& path, bool value)
{
    if (operationalStatus.find(path) == operationalStatus.end())
    {
        operationalStatus.emplace(path, std::make_unique<OperationalStatusIntf>(
                                            utils::getBus(), path.c_str()));
    }

    operationalStatus.at(path)->functional(value);
}

bool OccDBusSensors::getOperationalStatus(const std::string& path) const
{
    if (operationalStatus.find(path) != operationalStatus.end())
    {
        return operationalStatus.at(path)->functional();
    }

    throw std::invalid_argument("Failed to get OperationalStatus property.");
}

} // namespace dbus
} // namespace occ
} // namespace open_power
