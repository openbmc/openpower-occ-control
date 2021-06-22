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
bool OccDBusSensors::setMaxValue(const std::string& path, double value)
{
    if (path.empty())
    {
        return false;
    }

    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->maxValue(value);
    return true;
}

double OccDBusSensors::getMaxValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->maxValue();
    }

    throw std::invalid_argument("Failed to get MaxValue property.");
}

bool OccDBusSensors::setMinValue(const std::string& path, double value)
{
    if (path.empty())
    {
        return false;
    }

    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->minValue(value);
    return true;
}

double OccDBusSensors::getMinValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->minValue();
    }

    throw std::invalid_argument("Failed to get MinValue property.");
}

bool OccDBusSensors::setValue(const std::string& path, double value)
{
    if (path.empty())
    {
        return false;
    }

    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(
            path, std::make_unique<SensorIntf>(utils::getBus(), path.c_str()));
    }

    sensors.at(path)->value(value);
    return true;
}

double OccDBusSensors::getValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->value();
    }

    throw std::invalid_argument("Failed to get Value property.");
}

bool OccDBusSensors::setUnit(const std::string& path, const std::string& value)
{
    if (path.empty())
    {
        return false;
    }

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
        return false;
    }

    return true;
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

bool OccDBusSensors::setOperationalStatus(const std::string& path, bool value)
{
    if (path.empty())
    {
        return false;
    }

    if (operationalStatus.find(path) == operationalStatus.end())
    {
        operationalStatus.emplace(path, std::make_unique<OperationalStatusIntf>(
                                            utils::getBus(), path.c_str()));
    }

    operationalStatus.at(path)->functional(value);
    return true;
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
