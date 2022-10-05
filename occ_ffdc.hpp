#pragma once

#include "config.h"

#include "file.hpp"
#include "occ_errors.hpp"

#include <systemd/sd-journal.h>

#include <nlohmann/json.hpp>
#include <xyz/openbmc_project/Logging/Create/server.hpp>

using FFDCFormat =
    sdbusplus::xyz::openbmc_project::Logging::server::Create::FFDCFormat;
using FFDCFiles = std::vector<
    std::tuple<FFDCFormat, uint8_t, uint8_t, sdbusplus::message::unix_fd>>;

namespace open_power
{
namespace occ
{

/** @class FFDCFile
 *  @brief Represents a single file that will get opened when created and
 *         deleted when the object is destructed
 */
class FFDCFile
{
  public:
    FFDCFile() = delete;
    FFDCFile(const FFDCFile&) = delete;
    FFDCFile& operator=(const FFDCFile&) = delete;
    FFDCFile(FFDCFile&&) = delete;
    FFDCFile& operator=(FFDCFile&&) = delete;

    /**
     * @brief Constructor
     *
     * Opens the file and saves the descriptor
     *
     * @param[in] name - The filename
     */
    explicit FFDCFile(const std::filesystem::path& name);

    /**
     * @brief Destructor - Deletes the file
     */
    ~FFDCFile()
    {
        std::filesystem::remove(_name);
    }

    /**
     * @brief Returns the file descriptor
     *
     * @return int - The descriptor
     */
    int fd()
    {
        return _fd();
    }

  private:
    /**
     * @brief The file descriptor holder
     */
    FileDescriptor _fd;

    /**
     * @brief The filename
     */
    const std::filesystem::path _name;
};

/** @class FFDC
 *  @brief Monitors for SBE FFDC availability
 */
class FFDC : public Error
{
  public:
    FFDC() = delete;
    FFDC(const FFDC&) = delete;
    FFDC& operator=(const FFDC&) = delete;
    FFDC(FFDC&&) = default;
    FFDC& operator=(FFDC&&) = default;

    /** @brief Constructs the FFDC object
     *
     *  @param[in] event    - reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate FFDC data
     *  @param[in] instance - OCC instance number
     */
    FFDC(EventPtr& event, const fs::path& file, unsigned int instance) :
        Error(event, file, nullptr), instance(instance)
    {
        // Nothing to do here.
    }

    ~FFDC()
    {
        for (auto&& it : temporaryFiles)
        {
            close(it.second);
            fs::remove(it.first);
        }
    }

    /** @brief Helper function to create a PEL with the OpenPower DBus
     *         interface
     *
     * @param[in] path - the DBus error path
     * @param[in] src6 - the SBE error SRC6 word
     * @param[in] msg - the error message
     * @param[in] fd - the file descriptor for any FFDC
     */
    static uint32_t createPEL(const char* path, uint32_t src6, const char* msg,
                              int fd = -1);

    /** @brief Helper function to create a PEL for the OCC reset with the
     * OpenPower DBus interface
     *
     * @param[in] instance - the OCC instance id
     * @param[in] path - the DBus error path
     * @param[in] err - the error return code
     * @param[in] callout - the PEL callout path
     */
    static void createOCCResetPEL(unsigned int instance, const char* path,
                                  int err, const char* callout);

    /**
     * @brief Create a file containing the latest journal traces for the
     *        specified executable and add it to the file list.
     *
     * @param[in] fileList     - where to add the new file
     * @param[in] executable   - name of app to collect
     * @param[in] lines        - number of journal lines to save
     *
     * @return std::unique_ptr<FFDCFile> - The file object
     */
    static std::unique_ptr<FFDCFile>
        addJournalEntries(FFDCFiles& fileList, const std::string& executable,
                          unsigned int lines);

  private:
    /** @brief OCC instance number. Ex, 0,1, etc */
    unsigned int instance;

    /** @brief Stores the temporary files and file descriptors
     *         in usage. They will be cleaned up when the class
     *         is destroyed (when the application exits).
     */
    std::vector<std::pair<fs::path, int>> temporaryFiles;

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;

    /**
     * @brief Returns an FFDCFile containing the JSON data
     *
     * @param[in] ffdcData - The JSON data to write to a file
     *
     * @return std::unique_ptr<FFDCFile> - The file object
     */
    static std::unique_ptr<FFDCFile>
        makeJsonFFDCFile(const nlohmann::json& ffdcData);

    /**
     * @brief Returns a JSON structure containing the previous N journal
     * entries.
     *
     * @param[in] numLines   - Number of lines of journal to retrieve
     * @param[in] executable - name of app to collect for
     *
     * @return JSON object that was created
     */
    static nlohmann::json getJournalEntries(int numLines,
                                            std::string executable);

    /**
     * @brief Gets the realtime (wallclock) timestamp for the current journal
     * entry.
     *
     * @param journal current journal entry
     * @return timestamp as a date/time string
     */
    static std::string getTimeStamp(sd_journal* journal);

    /**
     * @brief Gets the value of the specified field for the current journal
     * entry.
     *
     * Returns an empty string if the current journal entry does not have the
     * specified field.
     *
     * @param journal current journal entry
     * @param field journal field name
     * @return field value
     */
    static std::string getFieldValue(sd_journal* journal,
                                     const std::string& field);
};

/**
 * @class JournalCloser
 *  @brief Automatically closes the journal when the object goes out of scope.
 */
class JournalCloser
{
  public:
    // Specify which compiler-generated methods we want
    JournalCloser() = delete;
    JournalCloser(const JournalCloser&) = delete;
    JournalCloser(JournalCloser&&) = delete;
    JournalCloser& operator=(const JournalCloser&) = delete;
    JournalCloser& operator=(JournalCloser&&) = delete;

    JournalCloser(sd_journal* journal) : journal{journal}
    {}

    ~JournalCloser()
    {
        sd_journal_close(journal);
    }

  private:
    sd_journal* journal{nullptr};
};

} // namespace occ
} // namespace open_power
