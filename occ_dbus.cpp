#include "occ_dbus.hpp"

#include <iostream>

namespace open_power
{
namespace occ
{
namespace dbus
{

void OccDBus::setMaxValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(path, std::make_unique<SensorIntf>(
                                  DBusHandler::getBus(), path.c_str()));
    }

    sensors.at(path)->maxValue(value);
}

double OccDBus::getMaxValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->maxValue();
    }

    return 0;
}

void OccDBus::setMinValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(path, std::make_unique<SensorIntf>(
                                  DBusHandler::getBus(), path.c_str()));
    }

    sensors.at(path)->minValue(value);
}

double OccDBus::getMinValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->minValue();
    }

    return 0;
}

void OccDBus::setValue(const std::string& path, double value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(path, std::make_unique<SensorIntf>(
                                  DBusHandler::getBus(), path.c_str()));
    }

    sensors.at(path)->value(value);
}

double OccDBus::getValue(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        return sensors.at(path)->value();
    }

    return 0;
}

void OccDBus::setUnit(const std::string& path, const std::string& value)
{
    if (sensors.find(path) == sensors.end())
    {
        sensors.emplace(path, std::make_unique<SensorIntf>(
                                  DBusHandler::getBus(), path.c_str()));
    }

    try
    {
        sensors.at(path)->unit(SensorIntf::convertUnitFromString(value));
    }
    catch (const std::exception& e)
    {
        std::cerr << "set Unit propety failed, e = " << e.what() << '\n';
    }
}

std::string OccDBus::getUnit(const std::string& path) const
{
    if (sensors.find(path) != sensors.end())
    {
        try
        {
            return SensorIntf::convertUnitToString(sensors.at(path)->unit());
        }
        catch (const std::exception& e)
        {
            std::cerr << "get Unit propety failed, e = " << e.what() << '\n';
        }
    }

    return "";
}

void OccDBus::setOperationalStatus(const std::string& path, bool value)
{
    if (operationalStatus.find(path) == operationalStatus.end())
    {
        operationalStatus.emplace(
            path, std::make_unique<OperationalStatusIntf>(DBusHandler::getBus(),
                                                          path.c_str()));
    }

    operationalStatus.at(path)->functional(value);
}

bool OccDBus::getOperationalStatus(const std::string& path) const
{
    if (operationalStatus.find(path) != operationalStatus.end())
    {
        return operationalStatus.at(path)->functional();
    }

    return false;
}

} // namespace dbus
} // namespace occ
} // namespace open_power
