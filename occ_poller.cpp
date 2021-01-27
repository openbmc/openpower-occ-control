#include "occ_poller.hpp"

#include "elog-errors.hpp"
#include "occ_manager.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <org/open_power/OCC/Device/error.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

namespace open_power
{
namespace occ
{

using namespace phosphor::logging;

void dump_hex(std::vector<std::int32_t> data, unsigned int data_len = 0)
{
    char line[64] = {0};
    char *c = line;
    unsigned int dump_length = data.size();
    if ((data_len > 0) && (data_len < dump_length)) dump_length = data_len;
    for (uint32_t i = 0; i < dump_length; i++)
    {
        if (i % 16 == 0)
        {
            sprintf(c, "%04X: ", i);
            c += 6;
        }
        else if (i % 4 == 0)
        {
            c[0] = ' ';
            c++;
        }

        sprintf(c, "%02X", data.at(i));
        c += 2;

        if ((i % 16 == 15) || (i == dump_length-1))
        {
            c[0] = '\0';
            log<level::INFO>(line);
            c = line;
        }
    }
}

void Poller::startPoller()
{
    std::string m = "Poller::startPoller() called";
    log<level::INFO>(m.c_str());

    // Start timer
    _pollTimer->restartOnce(std::chrono::seconds(interval));

}


void Poller::pollerTimerExpired()
{
    static bool L_traced = false;
    std::string m;
    if (manager.getNumOCCs() > 0)
    {
        //m = "Poller::pollerTimerExpired() called";
        //log<level::INFO>(m.c_str());

        if (!_pollTimer)
        {
            log<level::INFO>("Poller::pollerTimerExpired() not created");
            return;
        }

        // Send Poll
        m = "Poller::pollerTimerExpired() Send POLL to OCC" + std::to_string(occ_instance);
        log<level::INFO>(m.c_str());
        std::vector<std::int32_t> cmd, rsp;
        cmd.push_back(0x00); // Command (POLL)
        cmd.push_back(0x00); // Data Length (2 bytes)
        cmd.push_back(0x01);
        cmd.push_back(0x20); // Data (Version)
        m = "Poller::pollerTimerExpired() POLL command (" + std::to_string(cmd.size()) + " bytes)";
        log<level::INFO>(m.c_str());
        dump_hex(cmd);
        rsp = occ_pt->send(cmd);
        if (rsp.size() != 0)
        {
            m = "Poller::pollerTimerExpired() POLL response had " + std::to_string(rsp.size()) + " bytes";
            log<level::INFO>(m.c_str());
            dump_hex(rsp, 64);
        }
        else
        {
            log<level::INFO>("Poller::pollerTimerExpired() POLL response was empty");
        }

        // Restart Timer
        m = "Poller::pollerTimerExpired() restarting timer (" + std::to_string(interval) + " sec)";
        log<level::DEBUG>(m.c_str());
        _pollTimer->restartOnce(std::chrono::seconds(interval));

        if (L_traced) L_traced = false;
    }
    else if (!L_traced)
    {
        L_traced = true;
        log<level::INFO>("Poller::pollerTimerExpire() - No OCCs found");
    }
}


// Reads the occs_present file and analyzes the data
void Poller::analyzeEvent()
{
    using namespace phosphor::logging;
    using namespace sdbusplus::org::open_power::OCC::Device::Error;

    // Get the number of bytes to read
    int len = -1;
    auto r = ioctl(fd, FIONREAD, &len);
    if (r < 0)
    {
        std::string m = "Poller::analyzeEvent: OCC State Change Detected (failing ioctl: r=" + std::to_string(r) + ")";
        log<level::INFO>(m.c_str());
        elog<ConfigFailure>(
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_ERRNO(errno),
            phosphor::logging::org::open_power::OCC::Device::ConfigFailure::
                CALLOUT_DEVICE_PATH(file.c_str()));
    }

    //PassThrough pt = manager.passThroughObjects.front();

    auto data = readFile(len);
    if (data.empty())
    {
        log<level::INFO>("Poller::analyzeEvent: OCC State Change Detected (empty data)");
        return;
    }
    else
    {
        log<level::INFO>("Poller::analyzeEvent: OCC State Change Detected (normal exit)");
    }

#if 0
    if (callBack)
    {
        callBack(true);
    }
#endif
}


} // namespace occ
} // namespace open_power
