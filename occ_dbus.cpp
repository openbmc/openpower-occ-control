#include "occ_dbus.hpp"

#include "utils.hpp"

#include <phosphor-logging/log.hpp>

#include <format>
#include <iostream>

namespace open_power
{
namespace occ
{
namespace dbus
{

using namespace phosphor::logging;
using namespace std::string_literals;
const auto defaultChassisPath =
    "/xyz/openbmc_project/inventory/system/chassis"s;
const auto chassisInterface = "xyz.openbmc_project.Inventory.Item.Chassis"s;

bool OccDBusSensors::setMaxValue(const std::string& path, double value)
{
    if (path.empty())
    {
        return false;
    }

    if (!sensors.contains(path))
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

    if (!sensors.contains(path))
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

    if (!sensors.contains(path))
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

    if (!sensors.contains(path))
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

    if (!operationalStatus.contains(path))
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

void OccDBusSensors::setChassisAssociation(
    const std::string& path, const std::vector<std::string>& fTypes)
{
    using AssociationsEntry = std::tuple<std::string, std::string, std::string>;
    using AssociationsProperty = std::vector<AssociationsEntry>;
    using PropVariant = sdbusplus::xyz::openbmc_project::Association::server::
        Definitions::PropertiesVariant;

    if (chassisPath.empty())
    {
        chassisPath = getChassisPath();
    }

    AssociationsProperty associations;
    for (const auto& fType : fTypes)
    {
        associations.push_back(
            AssociationsEntry{"chassis", fType, chassisPath});
    }
    PropVariant value{std::move(associations)};

    std::map<std::string, PropVariant> properties;
    properties.emplace("Associations", std::move(value));

    chassisAssociations.emplace(
        path, std::make_unique<AssociationIntf>(utils::getBus(), path.c_str(),
                                                properties));
}

std::string OccDBusSensors::getChassisPath()
{
    try
    {
        auto paths = utils::getSubtreePaths(std::vector{chassisInterface});

        // For now, support either 1 chassis, or multiple as long as one
        // of them has the standard name, which we will use.  If this ever
        // fails, then someone would have to figure out how to identify the
        // chassis the OCCs are on.
        if (paths.size() == 1)
        {
            return paths[0];
        }
        else if (std::find(paths.begin(), paths.end(), defaultChassisPath) ==
                 paths.end())
        {
            log<level::ERR>(
                std::format(
                    "Could not find a chassis out of {} chassis objects",
                    paths.size())
                    .c_str());
            // Can't throw an exception here, the sdeventplus timer
            // just catches it.
            abort();
        }
    }
    catch (const std::exception& e)
    {
        log<level::ERR>(
            std::format("Error looking up chassis objects: {}", e.what())
                .c_str());
        abort();
    }

    return defaultChassisPath;
}

bool OccDBusSensors::hasDvfsTemp(const std::string& path) const
{
    return dvfsTemps.find(path) != dvfsTemps.end();
}

void OccDBusSensors::setDvfsTemp(const std::string& path, double value)
{
    dvfsTemps[path] =
        std::make_unique<SensorIntf>(utils::getBus(), path.c_str());
    dvfsTemps[path]->value(value);
}

} // namespace dbus
} // namespace occ
} // namespace open_power
