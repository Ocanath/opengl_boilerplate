#include "urdf_to_bullet/urdf_parser.h"

#include <tinyxml2.h>
#include <sstream>
#include <stdexcept>

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace urdf {
namespace {

void parseVec3Attr(const XMLElement* elem, const char* attrName, Vec3* out, Vec3 fallback = {})
{
    *out = fallback;
    if (!elem) return;
    const char* text = elem->Attribute(attrName);
    if (!text) return;

    std::istringstream ss(text);
    if (!(ss >> out->x >> out->y >> out->z))
        throw std::runtime_error(std::string("urdf: malformed \"") + attrName + "\" value: " + text);
}

void parsePose(const XMLElement* parent, Pose* out)
{
    *out = Pose{};
    if (!parent) return;
    const XMLElement* origin = parent->FirstChildElement("origin");
    if (!origin) return;
    parseVec3Attr(origin, "xyz", &out->xyz);
    parseVec3Attr(origin, "rpy", &out->rpy);
}

void parseGeometry(const XMLElement* geomParent, Geometry* out)
{
    *out = Geometry{};
    const XMLElement* geomElem = geomParent ? geomParent->FirstChildElement("geometry") : nullptr;
    if (!geomElem)
        throw std::runtime_error("urdf: missing <geometry> element");

    if (const XMLElement* box = geomElem->FirstChildElement("box")) {
        out->type = GeometryType::Box;
        parseVec3Attr(box, "size", &out->boxSize);
    } else if (const XMLElement* cyl = geomElem->FirstChildElement("cylinder")) {
        out->type = GeometryType::Cylinder;
        cyl->QueryDoubleAttribute("radius", &out->cylinderRadius);
        cyl->QueryDoubleAttribute("length", &out->cylinderLength);
    } else if (const XMLElement* sph = geomElem->FirstChildElement("sphere")) {
        out->type = GeometryType::Sphere;
        sph->QueryDoubleAttribute("radius", &out->sphereRadius);
    } else if (const XMLElement* mesh = geomElem->FirstChildElement("mesh")) {
        out->type = GeometryType::Mesh;
        if (const char* filename = mesh->Attribute("filename"))
            out->meshFilename = filename;
        parseVec3Attr(mesh, "scale", &out->meshScale, Vec3{1.0, 1.0, 1.0});
    } else {
        throw std::runtime_error("urdf: <geometry> has no box/cylinder/sphere/mesh child");
    }
}

void parseMaterial(const XMLElement* visualElem, Material* out)
{
    *out = Material{};
    const XMLElement* matElem = visualElem->FirstChildElement("material");
    if (!matElem) return;

    if (const char* name = matElem->Attribute("name"))
        out->name = name;

    if (const XMLElement* color = matElem->FirstChildElement("color")) {
        if (const char* rgba = color->Attribute("rgba")) {
            std::istringstream ss(rgba);
            if (ss >> out->r >> out->g >> out->b >> out->a)
                out->hasColor = true;
        }
    }
}

void parseInertial(const XMLElement* linkElem, Inertial* out)
{
    *out = Inertial{};
    const XMLElement* elem = linkElem->FirstChildElement("inertial");
    if (!elem) return;

    out->present = true;
    parsePose(elem, &out->origin);

    if (const XMLElement* mass = elem->FirstChildElement("mass"))
        mass->QueryDoubleAttribute("value", &out->mass);

    if (const XMLElement* inertia = elem->FirstChildElement("inertia")) {
        inertia->QueryDoubleAttribute("ixx", &out->ixx);
        inertia->QueryDoubleAttribute("iyy", &out->iyy);
        inertia->QueryDoubleAttribute("izz", &out->izz);
    }
}

} // namespace

// URDF files are small (tens to low hundreds of links); a linear scan by
// name is simpler than a lookup table and this only runs once at load time.
Link* findLinkByName(const std::vector<Link*>& links, const std::string& name)
{
    for (Link* link : links)
        if (link->name == name) return link;
    return nullptr;
}

void parseLink(const XMLElement* linkElem, Link* out)
{
    *out = Link{};
    if (const char* name = linkElem->Attribute("name"))
        out->name = name;

    parseInertial(linkElem, &out->inertial);

    for (const XMLElement* v = linkElem->FirstChildElement("visual"); v; v = v->NextSiblingElement("visual")) {
        Visual visual{};
        parsePose(v, &visual.origin);
        parseGeometry(v, &visual.geometry);
        parseMaterial(v, &visual.material);
        out->visuals.push_back(std::move(visual));
    }

    for (const XMLElement* c = linkElem->FirstChildElement("collision"); c; c = c->NextSiblingElement("collision")) {
        Collision collision{};
        parsePose(c, &collision.origin);
        parseGeometry(c, &collision.geometry);
        if (collision.geometry.type == GeometryType::Mesh) {
            throw std::runtime_error(
                "urdf: link \"" + out->name + "\" has a <collision> mesh geometry; "
                "urdf_to_bullet only maps box/cylinder/sphere collision primitives");
        }
        out->collisions.push_back(std::move(collision));
    }
}

void parseJoint(const XMLElement* jointElem, const std::vector<Link*>& links, Joint* out)
{
    *out = Joint{};
    if (const char* name = jointElem->Attribute("name"))
        out->name = name;

    const char* typeStr = jointElem->Attribute("type");
    std::string type = typeStr ? typeStr : "";
    if (type == "fixed")           out->type = JointType::Fixed;
    else if (type == "revolute")   out->type = JointType::Revolute;
    else if (type == "continuous") out->type = JointType::Continuous;
    else if (type == "prismatic")  out->type = JointType::Prismatic;
    else {
        throw std::runtime_error(
            "urdf: joint \"" + out->name + "\" has unsupported type \"" + type +
            "\" (urdf_to_bullet supports fixed/revolute/continuous/prismatic only)");
    }

    std::string parentName, childName;
    if (const XMLElement* parent = jointElem->FirstChildElement("parent"))
	{
        if (const char* link = parent->Attribute("link")) 
		{
			parentName = link;
		}
	}
    if (const XMLElement* child = jointElem->FirstChildElement("child"))
	{
		if (const char* link = child->Attribute("link")) 
		{
			childName = link;
		}
	}

    out->parentLink = findLinkByName(links, parentName);
    out->childLink   = findLinkByName(links, childName);
    if (!out->parentLink)
	{
		throw std::runtime_error("urdf: joint \"" + out->name + "\" references unknown parent link \"" + parentName + "\"");
	}
    if (!out->childLink)
    {
		throw std::runtime_error("urdf: joint \"" + out->name + "\" references unknown child link \"" + childName + "\"");
	}

    parsePose(jointElem, &out->origin);

    if (out->type != JointType::Fixed) 
	{
        parseVec3Attr(jointElem->FirstChildElement("axis"), "xyz", &out->axis, Vec3{1.0, 0.0, 0.0});

        if (const XMLElement* limit = jointElem->FirstChildElement("limit")) 
		{
            out->limit.present = true;
            limit->QueryDoubleAttribute("lower", &out->limit.lower);
            limit->QueryDoubleAttribute("upper", &out->limit.upper);
        }
    }
}

namespace {

Robot parseDocument(XMLDocument& doc)
{
    XMLElement* root = doc.RootElement();
    if (!root || std::string(root->Name()) != "robot")
        throw std::runtime_error("urdf: expected a <robot> root element");

    Robot robot{};
    if (const char* name = root->Attribute("name"))
        robot.name = name;

    for (const XMLElement* link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
        Link parsedLink{};
        parseLink(link, &parsedLink);
        robot.addLink(std::move(parsedLink));
    }

    for (const XMLElement* jointElem = root->FirstChildElement("joint"); jointElem; jointElem = jointElem->NextSiblingElement("joint")) {
        Joint parsedJoint{};
        parseJoint(jointElem, robot.links(), &parsedJoint);
        robot.addJoint(std::move(parsedJoint));
    }

    robot.finalize();
    return robot;
}

} // namespace

Robot parseUrdfFile(const std::string& path)
{
    XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("urdf: failed to load \"" + path + "\": " + doc.ErrorStr());
    return parseDocument(doc);
}

Robot parseUrdfString(const std::string& xml)
{
    XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error(std::string("urdf: failed to parse XML string: ") + doc.ErrorStr());
    return parseDocument(doc);
}

} // namespace urdf
