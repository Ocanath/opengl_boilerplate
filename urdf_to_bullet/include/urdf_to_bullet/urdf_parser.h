#pragma once
#include <string>
#include "urdf_to_bullet/urdf_model.h"

namespace urdf {

// Parses a URDF document into a Robot. Throws std::runtime_error on
// malformed XML, an unsupported joint type (floating/planar), or a
// <collision> geometry other than box/cylinder/sphere.
Robot parseUrdfFile(const std::string& path);
Robot parseUrdfString(const std::string& xml);

} // namespace urdf
