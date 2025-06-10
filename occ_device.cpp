#include "occ_device.hpp"

#include "occ_manager.hpp"
#include "occ_status.hpp"

#include <phosphor-logging/lg2.hpp>

#include <filesystem>
#include <iostream>

namespace open_power
{
namespace occ
{

void Device::setActive(bool active)
{
    std::string data = active ? "1" : "0";
    auto activeFile = devPath / "occ_active";
    try
    {
        write(activeFile, data);
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to set {DEVICE} active: {ERROR}", "DEVICE", devPath,
                   "ERROR", e.what());
    }
}

std::string Device::getPathBack(const fs::path& path)
{
    if (path.empty())
    {
        return std::string();
    }

    // Points to the last element in the path
    auto conf = --path.end();

    if (conf->empty() && conf != path.begin())
    {
        return *(--conf);
    }
    else
    {
        return *conf;
    }
}

bool Device::active() const
{
    return readBinary("occ_active");
}

bool Device::master() const
{
    return readBinary("occ_master");
}

bool Device::readBinary(const std::string& fileName) const
{
    int v = 0;
    if (statusObject.occActive())
    {
        auto filePath = devPath / fileName;
        std::ifstream file(filePath, std::ios::in);
        if (!file)
        {
            return false;
        }

        file >> v;
        file.close();
    }
    return v == 1;
}

void Device::errorCallback(int error)
{
    if (error)
    {
        if (error != -EHOSTDOWN)
        {
            fs::path p = devPath;
            if (fs::is_symlink(p))
            {
                p = fs::read_symlink(p);
            }
            statusObject.deviceError(
                Error::Descriptor("org.open_power.OCC.Device.Error.ReadFailure",
                                  error, p.c_str()));
        }
        else
        {
            statusObject.deviceError(Error::Descriptor(SAFE_ERROR_PATH));
        }
    }
}

void Device::presenceCallback(int)
{
    statusObject.deviceError(Error::Descriptor(PRESENCE_ERROR_PATH));
}

void Device::timeoutCallback(int error)
{
    if (error)
    {
        managerObject.sbeTimeout(instance);
    }
}

void Device::throttleProcTempCallback(int error)
{
    statusObject.throttleProcTemp(error);
    // Update the processor throttle on dbus
    statusObject.updateThrottle(error, THROTTLED_THERMAL);
}

void Device::throttleProcPowerCallback(int error)
{
    statusObject.throttleProcPower(error);
    // Update the processor throttle on dbus
    statusObject.updateThrottle(error, THROTTLED_POWER);
}

void Device::throttleMemTempCallback(int error)
{
    statusObject.throttleMemTemp(error);
}

fs::path Device::getFilenameByRegex(fs::path basePath,
                                    const std::regex& expr) const
{
    try
    {
        for (auto& file : fs::directory_iterator(basePath))
        {
            if (std::regex_search(file.path().string(), expr))
            {
                // Found match
                return file;
            }
        }
    }
    catch (const fs::filesystem_error& e)
    {
        lg2::error("getFilenameByRegex: Failed to get filename: {ERROR}",
                   "ERROR", e.what());
    }

    // Return empty path
    return fs::path{};
}

} // namespace occ
} // namespace open_power
