#include "occ_status.hpp"
namespace open_power
{
namespace occ
{

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value != this->occActive())
    {
        if (value)
        {
            // Bind the device
            device.bind();

            // And watch for errors
            device.addErrorWatch();
        }
        else
        {
            // Stop watching for errors
            device.removeErrorWatch();

            // Do the unbind.
            device.unBind();
        }
    }
    return Base::Status::occActive(value);
}

// Callback handler when a device error is reported.
void Status::deviceErrorHandler()
{
    // This would deem OCC inactive
    this->occActive(false);
}

} // namespace occ
} // namespace open_power
