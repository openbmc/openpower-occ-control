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
static constexpr auto loggingInterface = "org.open_power.Logging.PEL";

using namespace phosphor::logging;
using namespace sdbusplus::org::open_power::OCC::Device::Error;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

uint32_t FFDC::createPEL(const char* path, uint32_t src6, const char* msg,
                         int fd)
{
    uint32_t plid = 0;
    std::vector<std::tuple<
        sdbusplus::xyz::openbmc_project::Logging::server::Create::FFDCFormat,
        uint8_t, uint8_t, sdbusplus::message::unix_fd>>
        pelFFDCInfo;

    log<level::INFO>("Creating PEL with SBE FFDC", entry("SRC6=%08x", src6));

    if (fd > 0)
    {
        pelFFDCInfo.push_back(std::make_tuple(
            sdbusplus::xyz::openbmc_project::Logging::server::Create::
                FFDCFormat::Custom,
            static_cast<uint8_t>(0xCB), static_cast<uint8_t>(0x01), fd));
    }

    std::map<std::string, std::string> additionalData;
    additionalData.emplace("SRC6", std::to_string(src6));
    additionalData.emplace("_PID", std::to_string(getpid()));
    additionalData.emplace("SBE_ERR_MSG", msg);

    std::string service =
        utils::getService(loggingObjectPath, loggingInterface);
    auto& bus = utils::getBus();

    try
    {
        auto method =
            bus.new_method_call(service.c_str(), loggingObjectPath,
                                loggingInterface, "CreatePELWithFFDCFiles");
        auto level =
            sdbusplus::xyz::openbmc_project::Logging::server::convertForMessage(
                sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level::
                    Error);
        method.append(path, level, additionalData, pelFFDCInfo);
        auto response = bus.call(method);
        std::tuple<uint32_t, uint32_t> reply = {0, 0};

        response.read(reply);
        plid = std::get<1>(reply);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to create PEL");
    }

    return plid;
}

// Reads the FFDC file and create an error log
void FFDC::analyzeEvent()
{
    int tfd = -1;
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
        {
            break;
        }
        total += r;
    }

    lseek(fd, 0, SEEK_SET);

    uint32_t src6 = instance << 16;
    src6 |= *(data.get() + 2) << 8;
    src6 |= *(data.get() + 3);

    if (total > sbe_status_header_size)
    {
        std::string templateString =
            fs::temp_directory_path() / "OCC_FFDC_XXXXXX";
        tfd = mkostemp(templateString.data(), O_RDWR);
        if (tfd < 0)
        {
            log<level::ERR>("Couldn't create temporary FFDC file");
        }
        else
        {
            temporaryFiles.emplace_back(templateString, tfd);
            size_t written = sbe_status_header_size;
            while (written < total)
            {
                auto r = write(tfd, data.get() + written, total - written);
                if (r < 0)
                {
                    close(temporaryFiles.back().second);
                    fs::remove(temporaryFiles.back().first);
                    temporaryFiles.pop_back();
                    tfd = -1;
                    log<level::ERR>("Couldn't write temporary FFDC file");
                    break;
                }
                if (!r)
                {
                    break;
                }
                written += r;
            }
        }
    }

    createPEL("org.open_power.Processor.Error.SbeChipOpFailure", src6,
              "SBE command reported error", tfd);
}

} // namespace occ
} // namespace open_power
