#include "occ_status.hpp"
namespace open_power
{
namespace occ
{

// Handles updates to occActive property
bool Status::occActive(bool value)
{
    return Base::Status::occActive(value);
}

} // namespace occ
} // namespace open_power
