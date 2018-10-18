#pragma once

#include <systemd/sd-event.h>

#include <memory>
namespace open_power
{
namespace occ
{

/* Need a custom deleter for freeing up sd_event */
struct EventDeleter
{
    void operator()(sd_event* event) const
    {
        event = sd_event_unref(event);
    }
};
using EventPtr = std::unique_ptr<sd_event, EventDeleter>;

/* Need a custom deleter for freeing up sd_event_source */
struct EventSourceDeleter
{
    void operator()(sd_event_source* eventSource) const
    {
        eventSource = sd_event_source_unref(eventSource);
    }
};
using EventSourcePtr = std::unique_ptr<sd_event_source, EventSourceDeleter>;

} // namespace occ
} // namespace open_power
