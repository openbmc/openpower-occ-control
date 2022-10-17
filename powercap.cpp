#include "occ_status.hpp"

#include <phosphor-logging/lg2.hpp>
#include <powercap.hpp>

#include <cassert>
#include <filesystem>

namespace open_power
{
namespace occ
{
namespace powercap
{

constexpr auto PCAP_PATH = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";

namespace fs = std::filesystem;

using CapLimits =
    sdbusplus::xyz::openbmc_project::Control::Power::server::CapLimits;

// Print the current values
void OccPersistCapData::print()
{
    if (capData.initialized)
    {
        lg2::info(
            "OccPersistCapData: Soft Min: {SOFT}, Hard Min: {HARD}, Max: {MAX}",
            "SOFT", capData.softMin, "HARD", capData.hardMin, "MAX",
            capData.max);
    }
}

// Saves the power cap data in the filesystem using cereal.
void OccPersistCapData::save()
{
    std::filesystem::path opath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / powerCapFilename;

    if (!std::filesystem::exists(opath.parent_path()))
    {
        std::filesystem::create_directory(opath.parent_path());
    }

    std::ofstream stream{opath.c_str()};
    cereal::JSONOutputArchive oarchive{stream};

    oarchive(capData);
}

// Loads the power cap data in the filesystem using cereal.
void OccPersistCapData::load()
{
    std::filesystem::path ipath =
        std::filesystem::path{OCC_CONTROL_PERSIST_PATH} / powerCapFilename;

    if (!std::filesystem::exists(ipath))
    {
        capData.initialized = false;
        return;
    }

    try
    {
        std::ifstream stream{ipath.c_str()};
        cereal::JSONInputArchive iarchive(stream);
        iarchive(capData);
    }
    catch (const std::exception& e)
    {
        auto error = errno;
        lg2::error(
            "OccPersistCapData::load: failed to read {PATH}, errno={ERR}",
            "PATH", ipath.c_str(), "ERR", error);
        capData.initialized = false;
    }
}

void PowerCap::updatePcapBounds()
{
    // Build the hwmon string to write the power cap bounds
    fs::path minName = getPcapFilename(std::regex{"power\\d+_cap_min$"});
    fs::path softMinName =
        getPcapFilename(std::regex{"power\\d+_cap_min_soft$"});
    fs::path maxName = getPcapFilename(std::regex{"power\\d+_cap_max$"});

    // Read the current limits from persistent data
    uint32_t capSoftMin, capHardMin, capMax;
    persistedData.getCapLimits(capSoftMin, capHardMin, capMax);

    // Read the power cap bounds from sysfs files (from OCC)
    uint64_t cap;
    bool parmsChanged = false;
    std::ifstream softMinFile(softMinName, std::ios::in);
    if (softMinFile)
    {
        softMinFile >> cap;
        softMinFile.close();
        // Convert to input/AC Power in Watts (round up)
        capSoftMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find pcap_min_soft file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    std::ifstream minFile(minName, std::ios::in);
    if (minFile)
    {
        minFile >> cap;
        minFile.close();
        // Convert to input/AC Power in Watts (round up)
        capHardMin = ((cap / (PS_DERATING_FACTOR / 100.0) / 1000000) + 0.9);
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find cap_min file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    std::ifstream maxFile(maxName, std::ios::in);
    if (maxFile)
    {
        maxFile >> cap;
        maxFile.close();
        // Convert to input/AC Power in Watts (truncate remainder)
        capMax = cap / (PS_DERATING_FACTOR / 100.0) / 1000000;
        parmsChanged = true;
    }
    else
    {
        lg2::error(
            "updatePcapBounds: unable to find cap_max file: {FILE} (errno={ERR})",
            "FILE", pcapBasePathname, "ERR", errno);
    }

    if (parmsChanged)
    {
        // Save the power cap bounds to dbus
        updateDbusPcapLimits(capSoftMin, capHardMin, capMax);
        persistedData.updateCapLimits(capSoftMin, capHardMin, capMax);
    }

    // Validate user power cap (if enabled) is within the bounds
    const uint32_t dbusUserCap = getPcap();
    const bool pcapEnabled = getPcapEnabled();
    if (pcapEnabled && (dbusUserCap != 0))
    {
        const uint32_t hwmonUserCap = readUserCapHwmon();
        if ((dbusUserCap >= capSoftMin) && (dbusUserCap <= capMax))
        {
            // Validate dbus and hwmon user caps match
            if ((hwmonUserCap != 0) && (dbusUserCap != hwmonUserCap))
            {
                // User power cap is enabled, but does not match dbus
                lg2::error(
                    "updatePcapBounds: user powercap mismatch (hwmon:{HCAP}W, bdus:{DCAP}W) - using dbus",
                    "HCAP", hwmonUserCap, "DCAP", dbusUserCap);
                auto occInput = getOccInput(dbusUserCap, pcapEnabled);
                writeOcc(occInput);
            }
        }
        else
        {
            // User power cap is outside of current bounds
            uint32_t newCap = capMax;
            if (dbusUserCap < capSoftMin)
            {
                newCap = capSoftMin;
            }
            lg2::error(
                "updatePcapBounds: user powercap {CAP}W is outside bounds "
                "(soft min:{SMIN}, min:{MIN}, max:{MAX})",
                "CAP", dbusUserCap, "SMIN", capSoftMin, "MIN", capHardMin,
                "MAX", capMax);
            try
            {
                lg2::info(
                    "updatePcapBounds: Updating user powercap from {OLD} to {NEW}W",
                    "OLD", hwmonUserCap, "NEW", newCap);
                utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP,
                                   newCap);
                auto occInput = getOccInput(newCap, pcapEnabled);
                writeOcc(occInput);
            }
            catch (const sdbusplus::exception_t& e)
            {
                lg2::error(
                    "updatePcapBounds: Failed to update user powercap due to {ERR}",
                    "ERR", e.what());
            }
        }
    }
}

// Get value of power cap to send to the OCC (output/DC power)
uint32_t PowerCap::getOccInput(uint32_t pcap, bool pcapEnabled)
{
    if (!pcapEnabled)
    {
        // Pcap disabled, return 0 to indicate disabled
        return 0;
    }

    // If pcap is not disabled then just return the pcap with the derating
    // factor applied (output/DC power).
    return ((static_cast<uint64_t>(pcap) * PS_DERATING_FACTOR) / 100);
}

uint32_t PowerCap::getPcap()
{
    utils::PropertyValue pcap{};
    try
    {
        pcap = utils::getProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP);

        return std::get<uint32_t>(pcap);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Failed to get PowerCap property: path={PATH}: {ERR}",
                   "PATH", PCAP_PATH, "ERR", e.what());

        return 0;
    }
}

bool PowerCap::getPcapEnabled()
{
    utils::PropertyValue pcapEnabled{};
    try
    {
        pcapEnabled = utils::getProperty(PCAP_PATH, PCAP_INTERFACE,
                                         POWER_CAP_ENABLE_PROP);

        return std::get<bool>(pcapEnabled);
    }
    catch (const sdbusplus::exception_t& e)
    {
        lg2::error("Failed to get PowerCapEnable property: path={PATH}: {ERR}",
                   "PATH", PCAP_PATH, "ERR", e.what());

        return false;
    }
}

fs::path PowerCap::getPcapFilename(const std::regex& expr)
{
    if (pcapBasePathname.empty())
    {
        pcapBasePathname = occStatus.getHwmonPath();
    }

    if (fs::exists(pcapBasePathname))
    {
        // Search for pcap file based on the supplied expr
        for (auto& file : fs::directory_iterator(pcapBasePathname))
        {
            if (std::regex_search(file.path().string(), expr))
            {
                // Found match
                return file;
            }
        }
    }
    else
    {
        lg2::error("Power Cap base filename not found: {FILE}", "FILE",
                   pcapBasePathname);
    }

    // return empty path
    return fs::path{};
}

// Write the user power cap to sysfs (output/DC power)
// This will trigger the driver to send the cap to the OCC
void PowerCap::writeOcc(uint32_t pcapValue)
{
    if (!occStatus.occActive())
    {
        // OCC not running, skip update
        return;
    }

    // Build the hwmon string to write the user power cap
    fs::path fileName = getPcapFilename(std::regex{"power\\d+_cap_user$"});
    if (fileName.empty())
    {
        lg2::error("Could not find a power cap file to write to: {FILE})",
                   "FILE", pcapBasePathname);
        return;
    }

    uint64_t microWatts = pcapValue * 1000000ull;

    auto pcapString{std::to_string(microWatts)};

    // Open the hwmon file and write the power cap
    std::ofstream file(fileName, std::ios::out);
    if (file)
    {
        lg2::info("Writing {CAP}uW to {FILE}", "CAP", pcapString, "FILE",
                  fileName);
        file << pcapString;
        file.close();
    }
    else
    {
        lg2::error("Failed writing {CAP}uW to {FILE} (errno={ERR})", "CAP",
                   microWatts, "FILE", fileName, "ERR", errno);
    }

    return;
}

// Read the current user power cap from sysfs as input/AC power
uint32_t PowerCap::readUserCapHwmon()
{
    uint32_t userCap = 0;

    // Get the sysfs filename for the user power cap
    fs::path userCapName = getPcapFilename(std::regex{"power\\d+_cap_user$"});
    if (userCapName.empty())
    {
        lg2::error(
            "readUserCapHwmon: Could not find a power cap file to read: {FILE})",
            "FILE", pcapBasePathname);
        return 0;
    }

    // Open the sysfs file and read the power cap
    std::ifstream file(userCapName, std::ios::in);
    if (file)
    {
        uint64_t cap;
        file >> cap;
        file.close();
        // Convert to input/AC Power in Watts
        userCap = (cap / (PS_DERATING_FACTOR / 100.0) / 1000000);
    }
    else
    {
        lg2::error("readUserCapHwmon: Failed reading {FILE} (errno={ERR})",
                   "FILE", userCapName, "ERR", errno);
    }

    return userCap;
}

void PowerCap::pcapChanged(sdbusplus::message_t& msg)
{
    if (!occStatus.occActive())
    {
        // Nothing to do
        return;
    }

    uint32_t pcap = 0;
    bool pcapEnabled = false;

    std::string msgSensor;
    std::map<std::string, std::variant<uint32_t, bool>> msgData;
    msg.read(msgSensor, msgData);

    bool changeFound = false;
    for (const auto& [prop, value] : msgData)
    {
        if (prop == POWER_CAP_PROP)
        {
            pcap = std::get<uint32_t>(value);
            pcapEnabled = getPcapEnabled();
            changeFound = true;
        }
        else if (prop == POWER_CAP_ENABLE_PROP)
        {
            pcapEnabled = std::get<bool>(value);
            pcap = getPcap();
            changeFound = true;
        }
        else
        {
            // Ignore other properties
            lg2::debug(
                "pcapChanged: Unknown power cap property changed {PROP} to {VAL}",
                "PROP", prop, "VAL", std::get<uint32_t>(value));
        }
    }

    // Validate the cap is within supported range
    uint32_t capSoftMin, capHardMin, capMax;
    persistedData.getCapLimits(capSoftMin, capHardMin, capMax);
    if (((pcap > 0) && (pcap < capSoftMin)) || ((pcap == 0) && (pcapEnabled)))
    {
        lg2::error(
            "pcapChanged: Power cap of {CAP}W is lower than allowed (soft min:{SMIN}, min:{MIN}) - using soft min",
            "CAP", pcap, "SMIN", capSoftMin, "MIN", capHardMin);
        pcap = capSoftMin;
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP, pcap);
    }
    else if (pcap > capMax)
    {
        lg2::error(
            "pcapChanged: Power cap of {CAP}W is higher than allowed (max:{MAX}) - using max",
            "CAP", pcap, "MAX", capMax);
        pcap = capMax;
        utils::setProperty(PCAP_PATH, PCAP_INTERFACE, POWER_CAP_PROP, pcap);
    }

    if (changeFound)
    {
        lg2::info(
            "Power Cap Property Change (cap={CAP}W (input), enabled={ENABLE})",
            "CAP", pcap, "ENABLE", pcapEnabled);

        // Determine desired action to write to occ
        auto occInput = getOccInput(pcap, pcapEnabled);
        // Write action to occ
        writeOcc(occInput);
    }

    return;
}

// Update the Power Cap bounds on DBus
void PowerCap::updateDbusPcapLimits(uint32_t softMin, uint32_t hardMin,
                                    uint32_t max)
{
    CapLimits::minSoftPowerCapValue(softMin);
    CapLimits::minPowerCapValue(hardMin);
    CapLimits::maxPowerCapValue(max);
}

} // namespace powercap

} // namespace occ

} // namespace open_power
