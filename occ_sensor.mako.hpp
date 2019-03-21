## This file is a template.  The comment below is emitted
## into the rendered file; feel free to edit this file.
// WARNING: Generated header. Do not edit!


#pragma once

#include <map>
#include <string>
#include <tuple>

namespace open_power
{
namespace occ
{

using instanceID = int;
using sensorID = uint8_t;
using sensorName = std::string;
using sensorDefs = std::tuple<sensorID, sensorName>;
const std::map<instanceID, sensorDefs> Status::sensorMap = {
\
% for occ in occDict:
<%
    instance = occ.get("Instance")
    id = occ.get("SensorID")
    name = occ.get("SensorName")
%>\
\
    { ${instance}, { ${id}, "${name}" },\

% endfor
};

} // namespace occ
} // namespace open_power
