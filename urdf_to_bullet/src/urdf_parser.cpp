#include "urdf_to_bullet/urdf_parser.h"

#include <tinyxml2.h>
#include <sstream>
#include <stdexcept>

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace urdf {
namespace {

Vec3 parseVec3Attr(const XMLElement* elem, const char* attrName, Vec3 fallback = {})
{
    if (!elem) return fallback;
    const char* text = elem->Attribute(attrName);
    if (!text) return fallback;

    Vec3 v{};
    std::istringstream ss(text);
    if (!(ss >> v.x >> v.y >> v.z))
        throw std::runtime_error(std::string("urdf: malformed \"") + attrName + "\" value: " + text);
    return v;
}

Pose parsePose(const XMLElement* parent)
{
    Pose pose{};
    if (!parent) return pose;
    const XMLElement* origin = parent->FirstChildElement("origin");
    if (!origin) return pose;
    pose.xyz = parseVec3Attr(origin, "xyz");
    pose.rpy = parseVec3Attr(origin, "rpy");
    return pose;
}

Geometry parseGeometry(const XMLElement* geomParent)
{
    const XMLElement* geomElem = geomParent ? geomParent->FirstChildElement("geometry") : nullptr;
    if (!geomElem)
        throw std::runtime_error("urdf: missing <geometry> element");

    Geometry geom{};
    if (const XMLElement* box = geomElem->FirstChildElement("box")) {
        geom.type = GeometryType::Box;
        geom.boxSize = parseVec3Attr(box, "size");
    } else if (const XMLElement* cyl = geomElem->FirstChildElement("cylinder")) {
        geom.type = GeometryType::Cylinder;
        cyl->QueryDoubleAttribute("radius", &geom.cylinderRadius);
        cyl->QueryDoubleAttribute("length", &geom.cylinderLength);
    } else if (const XMLElement* sph = geomElem->FirstChildElement("sphere")) {
        geom.type = GeometryType::Sphere;
        sph->QueryDoubleAttribute("radius", &geom.sphereRadius);
    } else if (const XMLElement* mesh = geomElem->FirstChildElement("mesh")) {
        geom.type = GeometryType::Mesh;
        if (const char* filename = mesh->Attribute("filename"))
            geom.meshFilename = filename;
        geom.meshScale = parseVec3Attr(mesh, "scale", Vec3{1.0, 1.0, 1.0});
    } else {
        throw std::runtime_error("urdf: <geometry> has no box/cylinder/sphere/mesh child");
    }
    return geom;
}

Material parseMaterial(const XMLElement* visualElem)
{
    Material mat{};
    const XMLElement* matElem = visualElem->FirstChildElement("material");
    if (!matElem) return mat;

    if (const char* name = matElem->Attribute("name"))
        mat.name = name;

    if (const XMLElement* color = matElem->FirstChildElement("color")) {
        if (const char* rgba = color->Attribute("rgba")) {
            std::istringstream ss(rgba);
            if (ss >> mat.r >> mat.g >> mat.b >> mat.a)
                mat.hasColor = true;
        }
    }
    return mat;
}

Inertial parseInertial(const XMLElement* linkElem)
{
    Inertial inertial{};
    const XMLElement* elem = linkElem->FirstChildElement("inertial");
    if (!elem) return inertial;

    inertial.present = true;
    inertial.origin = parsePose(elem);

    if (const XMLElement* mass = elem->FirstChildElement("mass"))
        mass->QueryDoubleAttribute("value", &inertial.mass);

    if (const XMLElement* inertia = elem->FirstChildElement("inertia")) {
        inertia->QueryDoubleAttribute("ixx", &inertial.ixx);
        inertia->QueryDoubleAttribute("iyy", &inertial.iyy);
        inertia->QueryDoubleAttribute("izz", &inertial.izz);
    }
    return inertial;
}

Link parseLink(const XMLElement* linkElem)
{
    Link link{};
    if (const char* name = linkElem->Attribute("name"))
        link.name = name;

    link.inertial = parseInertial(linkElem);

    for (const XMLElement* v = linkElem->FirstChildElement("visual"); v; v = v->NextSiblingElement("visual")) {
        Visual visual{};
        visual.origin   = parsePose(v);
        visual.geometry = parseGeometry(v);
        visual.material = parseMaterial(v);
        link.visuals.push_back(std::move(visual));
    }

    for (const XMLElement* c = linkElem->FirstChildElement("collision"); c; c = c->NextSiblingElement("collision")) {
        Collision collision{};
        collision.origin   = parsePose(c);
        collision.geometry = parseGeometry(c);
        if (collision.geometry.type == GeometryType::Mesh) {
            throw std::runtime_error(
                "urdf: link \"" + link.name + "\" has a <collision> mesh geometry; "
                "urdf_to_bullet only maps box/cylinder/sphere collision primitives");
        }
        link.collisions.push_back(std::move(collision));
    }

    return link;
}

// URDF files are small (tens to low hundreds of links); a linear scan by
// name is simpler than a lookup table and this only runs once at load time.
Link* findLinkByName(const std::vector<Link*>& links, const std::string& name)
{
    for (Link* link : links)
        if (link->name == name) return link;
    return nullptr;
}

// Returned by value, same as parseLink(): built up as a local first so a
// thrown exception partway through just unwinds the stack. Robot::addJoint()
// takes ownership (heap-allocates it) and wires it into the pointer graph.
Joint parseJoint(const XMLElement* jointElem, const std::vector<Link*>& links)
{
    Joint joint{};
    if (const char* name = jointElem->Attribute("name"))
        joint.name = name;

    const char* typeStr = jointElem->Attribute("type");
    std::string type = typeStr ? typeStr : "";
    if (type == "fixed")           joint.type = JointType::Fixed;
    else if (type == "revolute")   joint.type = JointType::Revolute;
    else if (type == "continuous") joint.type = JointType::Continuous;
    else if (type == "prismatic")  joint.type = JointType::Prismatic;
    else {
        throw std::runtime_error(
            "urdf: joint \"" + joint.name + "\" has unsupported type \"" + type +
            "\" (urdf_to_bullet supports fixed/revolute/continuous/prismatic only)");
    }

    std::string parentName, childName;
    if (const XMLElement* parent = jointElem->FirstChildElement("parent"))
        if (const char* link = parent->Attribute("link")) parentName = link;
    if (const XMLElement* child = jointElem->FirstChildElement("child"))
        if (const char* link = child->Attribute("link")) childName = link;

    joint.parentLink = findLinkByName(links, parentName);
    joint.childLink   = findLinkByName(links, childName);
    if (!joint.parentLink)
        throw std::runtime_error("urdf: joint \"" + joint.name + "\" references unknown parent link \"" + parentName + "\"");
    if (!joint.childLink)
        throw std::runtime_error("urdf: joint \"" + joint.name + "\" references unknown child link \"" + childName + "\"");

    joint.origin = parsePose(jointElem);

    if (joint.type != JointType::Fixed) {
        joint.axis = parseVec3Attr(jointElem->FirstChildElement("axis"), "xyz", Vec3{1.0, 0.0, 0.0});

        if (const XMLElement* limit = jointElem->FirstChildElement("limit")) {
            joint.limit.present = true;
            limit->QueryDoubleAttribute("lower", &joint.limit.lower);
            limit->QueryDoubleAttribute("upper", &joint.limit.upper);
        }
    }

    return joint;
}

Robot parseDocument(XMLDocument& doc)
{
    XMLElement* root = doc.RootElement();
    if (!root || std::string(root->Name()) != "robot")
        throw std::runtime_error("urdf: expected a <robot> root element");

    Robot robot{};
    if (const char* name = root->Attribute("name"))
        robot.name = name;

    for (const XMLElement* link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
        robot.addLink(parseLink(link));

    for (const XMLElement* jointElem = root->FirstChildElement("joint"); jointElem; jointElem = jointElem->NextSiblingElement("joint"))
        robot.addJoint(parseJoint(jointElem, robot.links()));

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
