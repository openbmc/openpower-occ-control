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
        }
        else
        {
            // Do the unbind
            device.unBind();
        }
    }
    return Base::Status::occActive(value);
}

} // namespace occ
} // namespace open_power
