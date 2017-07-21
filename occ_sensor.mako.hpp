## This file is a template.  The comment below is emitted
## into the rendered file; feel free to edit this file.
// WARNING: Generated header. Do not edit!


#pragma once

#include <map>

namespace open_power
{
namespace occ
{

using instanceID = int;
using sensorID = uint8_t;
const std::map<instanceID, sensorID> Status::sensorMap = {
\
% for occ in occDict:
<%
    instance = occ.get("Instance")
    id = occ.get("SensorID")
%>\
\
    { ${instance}, ${id} },\

% endfor
};

} // namespace occ
} // namespace open_power
