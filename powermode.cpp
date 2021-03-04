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

    return mapperResponse.begin()->first;
}

uint32_t PowerMode::getMode()
{
    auto settingService = getService(PMODE_PATH, PMODE_INTERFACE);

    auto method =
        this->bus.new_method_call(settingService.c_str(), PMODE_PATH,
                                  "org.freedesktop.DBus.Properties", "Get");

    method.append(PMODE_INTERFACE, POWER_MODE_PROP);
    auto reply = this->bus.call(method);

    if (reply.is_method_error())
    {
        log<level::ERR>("Error in getMode prop");
        return 0;
    }
    std::variant<uint32_t> pmode;
    reply.read(pmode);

    return std::get<uint32_t>(pmode);
}

std::string PowerMode::getPmodeFilename(const fs::path& path)
{
    std::regex expr{"power\\d+_mode_user$"};
    for (auto& file : fs::directory_iterator(path))
    {
        if (std::regex_search(file.path().string(), expr))
        {
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

    std::string msgSensor;
    std::map<std::string, std::variant<uint32_t, bool>> msgData;
    msg.read(msgSensor, msgData);

    // Retrieve which property changed via the msg and read the other one
    auto valPropMap = msgData.find(POWER_MODE_PROP);
    if (valPropMap != msgData.end())
    {
        pmode = std::get<uint32_t>(valPropMap->second);
    }
    else
    {
        log<level::INFO>("Unknown power mode property changed");
        return;
    }

    log<level::INFO>(
        fmt::format("Power Mode Property Change: {}", pmode).c_str());

    // Determine desired action to write to occ

    // Write action to occ
    writeMode(pmode);

    return;
}

} // namespace powermode

} // namespace occ

} // namespace open_power
