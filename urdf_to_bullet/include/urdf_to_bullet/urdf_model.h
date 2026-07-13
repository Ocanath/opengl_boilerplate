#pragma once
#include <string>
#include <vector>

// Plain data representation of a parsed URDF document. No tinyxml2 or
// Bullet dependency here on purpose: urdf_parser.h fills this in from XML,
// urdf_to_bullet.h consumes it to build a Bullet simulation.
namespace urdf {

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

// A <origin xyz="..." rpy="..."/> element. rpy is fixed-axis roll-pitch-yaw
// in radians, applied as R = Rz(yaw) * Ry(pitch) * Rx(roll).
struct Pose {
    Vec3 xyz;
    Vec3 rpy;
};

enum class GeometryType { Box, Cylinder, Sphere, Mesh };

struct Geometry {
    GeometryType type = GeometryType::Box;

    Vec3   boxSize;              // Box: full extents (x, y, z)
    double cylinderRadius = 0.0; // Cylinder: radius, axis along local Z
    double cylinderLength = 0.0; // Cylinder: full length along local Z
    double sphereRadius   = 0.0; // Sphere: radius

    std::string meshFilename;    // Mesh: path as written in the URDF (unresolved)
    Vec3        meshScale{1.0, 1.0, 1.0};
};

struct Material {
    std::string name;
    bool        hasColor = false;
    float       r = 1.f, g = 1.f, b = 1.f, a = 1.f;
};

// <visual> is render-only: never consumed by urdf_to_bullet's physics build,
// only carried through so a renderer can use it.
struct Visual {
    Pose     origin;
    Geometry geometry;
    Material material;
};

// <collision>: urdf_to_bullet only maps Box/Cylinder/Sphere geometry here to
// Bullet primitive shapes; a Mesh collision geometry is rejected at parse time.
struct Collision {
    Pose     origin;
    Geometry geometry;
};

struct Inertial {
    bool   present = false;
    Pose   origin;   // center of mass, relative to the link frame
    double mass = 0.0;
    // Diagonal terms only. Bullet's rigid body inertia tensor is diagonal in
    // the body's local frame; off-diagonal products of inertia (ixy/ixz/iyz)
    // have no equivalent there and are discarded.
    double ixx = 0.0, iyy = 0.0, izz = 0.0;
};

struct Link {
    std::string            name;
    Inertial                inertial;
    std::vector<Visual>     visuals;
    std::vector<Collision>  collisions;
};

enum class JointType { Fixed, Revolute, Continuous, Prismatic };

struct JointLimit {
    bool   present = false;
    double lower = 0.0;
    double upper = 0.0;
};

struct Joint {
    std::string name;
    JointType   type = JointType::Fixed;
    std::string parentLink;
    std::string childLink;
    Pose        origin;            // child frame, relative to parent link frame
    Vec3        axis{1.0, 0.0, 0.0}; // revolute/continuous/prismatic axis, in the joint frame
    JointLimit  limit;              // revolute: rotation range; prismatic: translation range
};

struct Robot {
    std::string        name;
    std::vector<Link>  links;
    std::vector<Joint> joints;
};

} // namespace urdf
