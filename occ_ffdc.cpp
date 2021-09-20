#include "occ_ffdc.hpp"

#include "elog-errors.hpp"
#include "utils.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Logging/Create/server.hpp>

namespace open_power
{
namespace occ
{

static constexpr size_t max_ffdc_size = 8192;
static constexpr size_t sbe_status_header_size = 8;

static constexpr auto loggingObjectPath = "/xyz/openbmc_project/logging";
static constexpr auto loggingInterface = "xyz.openbmc_project.Logging.Create";

// Reads the FFDC file and create an error log
void FFDC::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;
    using InternalFailure =
        sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

    size_t total = 0;
    auto data = std::make_unique<unsigned char[]>(max_ffdc_size);
    while (total < max_ffdc_size)
    {
        auto r = read(fd, data.get() + total, max_ffdc_size - total);
        if (r < 0)
        {
            elog<ReadFailure>(
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_ERRNO(errno),
                phosphor::logging::org::open_power::OCC::Device::ReadFailure::
                    CALLOUT_DEVICE_PATH(file.c_str()));
            return;
        }
        if (!r)
            break;
        total += r;
    }

    lseek(fd, 0, SEEK_SET);

    if (total < sbe_status_header_size)
        return;

    uint32_t src6 = instance << 16;
    src6 |= *(data.get() + 2) << 8;
    src6 |= *(data.get() + 3);

    std::string templateString = fs::temp_directory_path() / "OCC_FFDC_XXXXXX";
    int tfd = mkostemp(templateString.data(), O_RDWR);
    if (tfd < 0)
    {
        log<level::ERR>("Couldn't create temporary FFDC file");
        elog<InternalFailure>();
    }

    temporaryFiles.emplace_back(templateString, tfd);
    size_t written = sbe_status_header_size; // skip the SBE response header
    while (written < total)
    {
        auto r = write(tfd, data.get() + written, total - written);
        if (r < 0)
        {
            close(temporaryFiles.back().second);
            fs::remove(temporaryFiles.back().first);
            temporaryFiles.pop_back();
            log<level::ERR>("Couldn't write temporary FFDC file");
            elog<InternalFailure>();
        }
        if (!r)
            break;
        written += r;
    }

    std::vector<std::tuple<
        sdbusplus::xyz::openbmc_project::Logging::server::Create::FFDCFormat,
        uint8_t, uint8_t, sdbusplus::message::unix_fd>>
        pelFFDCInfo;
    pelFFDCInfo.push_back(std::make_tuple(
        sdbusplus::xyz::openbmc_project::Logging::server::Create::FFDCFormat::
            Custom,
        static_cast<uint8_t>(0xCB), static_cast<uint8_t>(0x01), tfd));

    std::map<std::string, std::string> additionalData;
    additionalData.emplace("SRC6", std::to_string(src6));
    additionalData.emplace("_PID", std::to_string(getpid()));
    additionalData.emplace("SBE_ERR_MSG", "SBE command reported error");

    std::string service =
        utils::getService(loggingObjectPath, loggingInterface);
    auto& bus = utils::getBus();

    try
    {
        auto method =
            bus.new_method_call(service.c_str(), loggingObjectPath,
                                loggingInterface, "CreateWithFFDCFiles");
        auto level =
            sdbusplus::xyz::openbmc_project::Logging::server::convertForMessage(
                sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level::
                    Error);
        method.append("org.open_power.Processor.Error.SbeChipOpFailure", level,
                      additionalData, pelFFDCInfo);
        auto resp = bus.call(method);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to create PEL");
    }
}

} // namespace occ
} // namespace open_power
