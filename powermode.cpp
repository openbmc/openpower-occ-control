#include <fmt/core.h>

#include <cassert>
#include <phosphor-logging/log.hpp>
#include <powermode.hpp>
#include <regex>

namespace open_power
{
namespace occ
{
namespace powermode
{

constexpr auto PMODE_PATH = "/xyz/openbmc_project/control/host0/power_mode";
constexpr auto PMODE_INTERFACE = "xyz.openbmc_project.Control.Power.Mode";

constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

constexpr auto POWER_MODE_PROP = "PowerMode";

using namespace phosphor::logging;
namespace fs = std::experimental::filesystem;

std::string PowerMode::getService(std::string path, std::string interface)
{
    auto mapper = bus.new_method_call(MAPPER_BUSNAME, MAPPER_PATH,
                                      MAPPER_INTERFACE, "GetObject");

    mapper.append(path, std::vector<std::string>({interface}));
    auto mapperResponseMsg = bus.call(mapper);

    if (mapperResponseMsg.is_method_error())
    {
        log<level::ERR>(
            fmt::format("Error in mapper call (path: {}, interface: {}", path,
                        interface)
                .c_str());
        throw std::runtime_error("Error in mapper call");
    }

    std::map<std::string, std::vector<std::string>> mapperResponse;
    mapperResponseMsg.read(mapperResponse);
    if (mapperResponse.empty())
    {
        log<level::ERR>(
            fmt::format(
                "Error reading mapper response (path: {}, interface: {}", path,
                interface)
                .c_str());
        throw std::runtime_error("Error reading mapper response");
    }
    log<level::INFO>(
        fmt::format("PowerMode::getService(path: {}, interface: {})", path,
                    interface)
            .c_str());

    return mapperResponse.begin()->first;
}

std::string PowerMode::getPmodeFilename(const fs::path& path)
{
    std::regex expr{"power\\d+_mode_user$"};
    for (auto& file : fs::directory_iterator(path))
    {
        if (std::regex_search(file.path().string(), expr))
        {
            log<level::INFO>(
                fmt::format("PowerMode::getPmodeFilename returning {}",
                            file.path().filename().c_str())
                    .c_str());

            return file.path().filename();
        }
    }
    return std::string{};
}

void PowerMode::writeMode(uint8_t pmodeValue)
{
    // Create path out to master occ hwmon entry
    std::unique_ptr<fs::path> fileName =
        std::make_unique<fs::path>(OCC_HWMON_PATH);
    *fileName /= occMasterName;
    *fileName /= "/hwmon/";

    // Need to get the hwmonXX directory name, there better only be 1 dir
    assert(std::distance(fs::directory_iterator(*fileName),
                         fs::directory_iterator{}) == 1);
    // Now set our path to this full path, including this hwmonXX directory
    fileName = std::make_unique<fs::path>(*fs::directory_iterator(*fileName));
    // Append on the hwmon string where we write the user power mode

    auto baseName = getPmodeFilename(*fileName);
    if (baseName.empty())
    {
        log<level::ERR>(
            fmt::format(
                "Could not find a power mode file to write to (path: {})",
                fileName->c_str())
                .c_str());
        return;
    }
    *fileName /= baseName;

    log<level::INFO>(
        fmt::format("Writing pmode value to hwmon (path: {}, mode: 0x{:02X})",
                    fileName->c_str(), pmodeValue)
            .c_str());
    // Open the hwmon file and write the power mode
    std::ofstream file(*fileName, std::ios::out);
    file << std::to_string(pmodeValue);
    file.close();
    return;
}

void PowerMode::modeChanged(sdbusplus::message::message& msg)
{
    if (!occStatus.occActive())
    {
        // Nothing to  do
        return;
    }

    uint8_t pmode = 0;

    std::map<std::string, std::variant<std::string>> properties{};
    std::string interface;
    msg.read(interface, properties);
    const auto stateEntry = properties.find(POWER_MODE_PROP);
    if (stateEntry != properties.end())
    {
        auto stateEntryValue = stateEntry->second;
        auto propVal = std::get<std::string>(stateEntryValue);
        if (propVal == "xyz.openbmc_project.Control.Power.Mode.PowerMode."
                       "MaximumPerformance")
        {
            log<level::INFO>("PowerMode(MaxPerf)");
            pmode = 12;
        }
        else if (propVal ==
                 "xyz.openbmc_project.Control.Power.Mode.PowerMode.PowerSaving")
        {
            log<level::INFO>("PowerMode(PowerSaving)");
            pmode = 5;
        }
        else if (propVal ==
                 "xyz.openbmc_project.Control.Power.Mode.PowerMode.Static")
        {
            log<level::INFO>("PowerMode(Static)");
            pmode = 1;
        }
        else if (propVal ==
                 "xyz.openbmc_project.Control.Power.Mode.PowerMode.OEM")
        {
            log<level::INFO>("PowerMode(OEM)");
            pmode = 10;
        }
        else
        {
            log<level::ERR>(
                fmt::format("modeChanged: Invalid PowerMode specified: {}",
                            propVal)
                    .c_str());
        }
        log<level::INFO>(
            fmt::format("modeChanged: {} -> PowerMode({})", propVal, pmode)
                .c_str());
    }

    log<level::INFO>(
        fmt::format("Power Mode Property Change: {}", pmode).c_str());

    // Trigger mode change to OCC
    occStatus.occsWentActive();

    return;
}

} // namespace powermode

} // namespace occ

} // namespace open_power
