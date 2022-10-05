#include "occ_ffdc.hpp"

#include "elog-errors.hpp"
#include "utils.hpp"

#include <errno.h>
#include <fcntl.h>
#include <fmt/core.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
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
static constexpr auto opLoggingInterface = "org.open_power.Logging.PEL";

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

    log<level::INFO>(
        fmt::format("Creating PEL for OCC{} with SBE FFDC: {} - SRC6: 0x{:08X}",
                    src6 >> 16, path, src6)
            .c_str());

    if (fd > 0)
    {
        pelFFDCInfo.push_back(std::make_tuple(
            sdbusplus::xyz::openbmc_project::Logging::server::Create::
                FFDCFormat::Custom,
            static_cast<uint8_t>(0xCB), static_cast<uint8_t>(0x01), fd));
    }

    // Add journal traces to PEL FFDC
    auto occJournalFile =
        addJournalEntries(pelFFDCInfo, "openpower-occ-control", 25);

    std::map<std::string, std::string> additionalData;
    additionalData.emplace("SRC6", std::to_string(src6));
    additionalData.emplace("_PID", std::to_string(getpid()));
    additionalData.emplace("SBE_ERR_MSG", msg);

    auto& bus = utils::getBus();

    try
    {
        std::string service =
            utils::getService(loggingObjectPath, opLoggingInterface);
        auto method =
            bus.new_method_call(service.c_str(), loggingObjectPath,
                                opLoggingInterface, "CreatePELWithFFDCFiles");

        // Set level to Notice (Informational). Error should trigger an OCC
        // reset and if it does not recover, HTMGT/HBRT will create an
        // unrecoverable error.
        auto level =
            sdbusplus::xyz::openbmc_project::Logging::server::convertForMessage(
                sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level::
                    Notice);

        method.append(path, level, additionalData, pelFFDCInfo);
        auto response = bus.call(method);
        std::tuple<uint32_t, uint32_t> reply = {0, 0};

        response.read(reply);
        plid = std::get<1>(reply);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            fmt::format("Failed to create PEL: {}", e.what()).c_str());
    }

    return plid;
}

void FFDC::createOCCResetPEL(unsigned int instance, const char* path, int err,
                             const char* callout)
{
    std::map<std::string, std::string> additionalData;

    additionalData.emplace("_PID", std::to_string(getpid()));

    if (err)
    {
        additionalData.emplace("CALLOUT_ERRNO", std::to_string(-err));
    }

    if (callout)
    {
        additionalData.emplace("CALLOUT_DEVICE_PATH", std::string(callout));
    }

    additionalData.emplace("OCC", std::to_string(instance));

    log<level::INFO>(
        fmt::format("Creating OCC Reset PEL for OCC{}: {}", instance, path)
            .c_str());

    auto& bus = utils::getBus();

    try
    {
        FFDCFiles ffdc;
        // Add journal traces to PEL FFDC
        auto occJournalFile =
            addJournalEntries(ffdc, "openpower-occ-control", 25);

        std::string service =
            utils::getService(loggingObjectPath, opLoggingInterface);
        auto method =
            bus.new_method_call(service.c_str(), loggingObjectPath,
                                opLoggingInterface, "CreatePELWithFFDCFiles");

        // Set level to Notice (Informational). Error should trigger an OCC
        // reset and if it does not recover, HTMGT/HBRT will create an
        // unrecoverable error.
        auto level =
            sdbusplus::xyz::openbmc_project::Logging::server::convertForMessage(
                sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level::
                    Notice);

        method.append(path, level, additionalData, ffdc);
        bus.call(method);
    }
    catch (const sdbusplus::exception_t& e)
    {
        log<level::ERR>(
            fmt::format("Failed to create OCC Reset PEL: {}", e.what())
                .c_str());
    }
}

// Reads the SBE FFDC file and create an error log
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

    if (!total)
    {
        // no error
        return;
    }

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

// Create file with the latest journal entries for specified executable
std::unique_ptr<FFDCFile> FFDC::addJournalEntries(FFDCFiles& fileList,
                                                  const std::string& executable,
                                                  unsigned int lines)
{
    auto journalFile = makeJsonFFDCFile(getJournalEntries(lines, executable));
    if (journalFile && journalFile->fd() != -1)
    {
        log<level::DEBUG>(
            fmt::format(
                "addJournalEntries: Added up to {} journal entries for {}",
                lines, executable)
                .c_str());
        fileList.emplace_back(FFDCFormat::JSON, 0x01, 0x01, journalFile->fd());
    }
    else
    {
        log<level::ERR>(
            fmt::format(
                "addJournalEntries: Failed to add journal entries for {}",
                executable)
                .c_str());
    }
    return journalFile;
}

// Write JSON data into FFDC file and return the file
std::unique_ptr<FFDCFile> FFDC::makeJsonFFDCFile(const nlohmann::json& ffdcData)
{
    std::string tmpFile = fs::temp_directory_path() / "OCC_JOURNAL_XXXXXX";
    auto fd = mkostemp(tmpFile.data(), O_RDWR);
    if (fd != -1)
    {
        auto jsonString = ffdcData.dump();
        auto rc = write(fd, jsonString.data(), jsonString.size());
        close(fd);
        if (rc != -1)
        {
            fs::path jsonFile{tmpFile};
            return std::make_unique<FFDCFile>(jsonFile);
        }
        else
        {
            auto e = errno;
            log<level::ERR>(
                fmt::format(
                    "makeJsonFFDCFile: Failed call to write JSON FFDC file, errno={}",
                    e)
                    .c_str());
        }
    }
    else
    {
        auto e = errno;
        log<level::ERR>(
            fmt::format("makeJsonFFDCFile: Failed called to mkostemp, errno={}",
                        e)
                .c_str());
    }
    return nullptr;
}

// Collect the latest journal entries for a specified executable
nlohmann::json FFDC::getJournalEntries(int numLines, std::string executable)
{
    // Sleep 100ms; otherwise recent journal entries sometimes not available
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(100ms);

    std::vector<std::string> entries;

    // Open the journal
    sd_journal* journal;
    int rc = sd_journal_open(&journal, SD_JOURNAL_LOCAL_ONLY);
    if (rc < 0)
    {
        // Build one line string containing field values
        entries.push_back("[Internal error: sd_journal_open(), rc=" +
                          std::string(strerror(rc)) + "]");
        return nlohmann::json(entries);
    }

    // Create object to automatically close journal
    JournalCloser closer{journal};

    // Add match so we only loop over entries with specified field value
    std::string field{"SYSLOG_IDENTIFIER"};
    std::string match{field + '=' + executable};
    rc = sd_journal_add_match(journal, match.c_str(), 0);
    if (rc < 0)
    {
        // Build one line string containing field values
        entries.push_back("[Internal error: sd_journal_add_match(), rc=" +
                          std::string(strerror(rc)) + "]");
    }
    else
    {
        int count{1};
        entries.reserve(numLines);
        std::string syslogID, pid, message, timeStamp;

        // Loop through journal entries from newest to oldest
        SD_JOURNAL_FOREACH_BACKWARDS(journal)
        {
            // Get relevant journal entry fields
            timeStamp = getTimeStamp(journal);
            syslogID = getFieldValue(journal, "SYSLOG_IDENTIFIER");
            pid = getFieldValue(journal, "_PID");
            message = getFieldValue(journal, "MESSAGE");

            // Build one line string containing field values
            entries.push_back(timeStamp + " " + syslogID + "[" + pid +
                              "]: " + message);

            // Stop after number of lines was read
            if (count++ >= numLines)
            {
                break;
            }
        }
    }

    // put the journal entries in chronological order
    std::reverse(entries.begin(), entries.end());

    return nlohmann::json(entries);
}

std::string FFDC::getTimeStamp(sd_journal* journal)
{
    // Get realtime (wallclock) timestamp of current journal entry.  The
    // timestamp is in microseconds since the epoch.
    uint64_t usec{0};
    int rc = sd_journal_get_realtime_usec(journal, &usec);
    if (rc < 0)
    {
        return "[Internal error: sd_journal_get_realtime_usec(), rc=" +
               std::string(strerror(rc)) + "]";
    }

    // Convert to number of seconds since the epoch
    time_t secs = usec / 1000000;

    // Convert seconds to tm struct required by strftime()
    struct tm* timeStruct = localtime(&secs);
    if (timeStruct == nullptr)
    {
        return "[Internal error: localtime() returned nullptr]";
    }

    // Convert tm struct into a date/time string
    char timeStamp[80];
    strftime(timeStamp, sizeof(timeStamp), "%b %d %H:%M:%S", timeStruct);

    return timeStamp;
}

std::string FFDC::getFieldValue(sd_journal* journal, const std::string& field)
{
    std::string value{};

    // Get field data from current journal entry
    const void* data{nullptr};
    size_t length{0};
    int rc = sd_journal_get_data(journal, field.c_str(), &data, &length);
    if (rc < 0)
    {
        if (-rc == ENOENT)
        {
            // Current entry does not include this field; return empty value
            return value;
        }
        else
        {
            return "[Internal error: sd_journal_get_data() rc=" +
                   std::string(strerror(rc)) + "]";
        }
    }

    // Get value from field data.  Field data in format "FIELD=value".
    std::string dataString{static_cast<const char*>(data), length};
    std::string::size_type pos = dataString.find('=');
    if ((pos != std::string::npos) && ((pos + 1) < dataString.size()))
    {
        // Value is substring after the '='
        value = dataString.substr(pos + 1);
    }

    return value;
}

// Create temporary file that will automatically get removed when destructed
FFDCFile::FFDCFile(const fs::path& name) :
    _fd(open(name.c_str(), O_RDONLY)), _name(name)
{
    if (_fd() == -1)
    {
        auto e = errno;
        log<level::ERR>(
            fmt::format("FFDCFile: Could not open FFDC file {}. errno {}",
                        _name.string(), e)
                .c_str());
    }
}

} // namespace occ
} // namespace open_power
