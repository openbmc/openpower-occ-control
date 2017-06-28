#include "occ_status.hpp"
namespace open_power
{
namespace occ
{

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    if (value)
    {
        // Bind the device
        device.bind();
    }
    else
    {
        // Do the unbind. Although making the OCC inactive
        // should not be a user triggered operation, this action
        // can not be stopped so need to unbind
        device.unBind();
    }
    return Base::Status::occActive(value);
}

} // namespace occ
} // namespace open_power
