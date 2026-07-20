#pragma once
#include <string>
#include <vector>
#include "urdf_to_bullet/urdf_model.h"

namespace tinyxml2 { class XMLElement; }

namespace urdf {

// Parses a URDF document into a Robot. Throws std::runtime_error on
// malformed XML, an unsupported joint type (floating/planar), or a
// <collision> geometry other than box/cylinder/sphere.
Robot parseUrdfFile(const std::string& path);
Robot parseUrdfString(const std::string& xml);

// Fill *out from a single <link>/<joint> element. Exposed (rather than kept
// file-local) so callers that own their Link*/Joint* storage directly, e.g.
// DynamicRobot, can parse into it without going through a Robot. Throws
// std::runtime_error on the same malformed input as parseUrdfFile/String.
// parseJoint looks up parent/child by name in `links`, same as Robot::links().
void parseLink(const tinyxml2::XMLElement* linkElem, Link* out);
void parseJoint(const tinyxml2::XMLElement* jointElem, const std::vector<Link*>& links, Joint* out);

} // namespace urdf
