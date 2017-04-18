#include <org/open_power/OCC/PassThrough/error.hpp>

namespace sdbusplus
{
namespace org
{
namespace open_power
{
namespace OCC
{
namespace PassThrough
{
namespace Error
{
const char* OpenFailure::name() const noexcept
{
    return errName;
}
const char* OpenFailure::description() const noexcept
{
    return errDesc;
}
const char* OpenFailure::what() const noexcept
{
    return errWhat;
}
const char* ReadFailure::name() const noexcept
{
    return errName;
}
const char* ReadFailure::description() const noexcept
{
    return errDesc;
}
const char* ReadFailure::what() const noexcept
{
    return errWhat;
}
const char* WriteFailure::name() const noexcept
{
    return errName;
}
const char* WriteFailure::description() const noexcept
{
    return errDesc;
}
const char* WriteFailure::what() const noexcept
{
    return errWhat;
}

} // namespace Error
} // namespace PassThrough
} // namespace OCC
} // namespace open_power
} // namespace org
} // namespace sdbusplus

