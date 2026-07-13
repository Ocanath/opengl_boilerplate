#pragma once
#include <stdexcept>
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

struct Joint; // Link::joints references Joint before it's declared below.

// A node in the robot's kinematic tree.
struct Link {
    std::string             name;
    Inertial                 inertial;
    std::vector<Visual>      visuals;
    std::vector<Collision>   collisions;

    // Every joint touching this link, as either parent or child (an edge
    // list, not just the edges going "down" to children) — walk the tree by
    // checking `joint->parentLink == this` to find this link's children.
    std::vector<Joint*> joints;

    // Set by the parser when some joint's <child> names this link. The link
    // for which this is still false after parsing is the tree's root.
    bool hasParentJoint = false;
};

enum class JointType { Fixed, Revolute, Continuous, Prismatic };

struct JointLimit {
    bool   present = false;
    double lower = 0.0;
    double upper = 0.0;
};

// An edge in the robot's kinematic tree.
struct Joint {
    std::string name;
    JointType   type = JointType::Fixed;
    Link*       parentLink = nullptr;
    Link*       childLink  = nullptr;
    Pose        origin;              // child frame, relative to parent link frame
    Vec3        axis{1.0, 0.0, 0.0}; // revolute/continuous/prismatic axis, in the joint frame
    JointLimit  limit;                // revolute: rotation range; prismatic: translation range
};

// Owns every Link and Joint by pointer. Each one is individually
// heap-allocated so that Joint::parentLink/childLink and Link::joints stay
// valid while the internal vectors grow and reallocate during parsing —
// growing a vector of pointers only moves the pointers, never the Link/Joint
// objects they point at.
//
// The only way in is addLink()/addJoint()/finalize() (used by urdf_parser),
// and there is deliberately no way to remove a single node: reaching in and
// erasing one link or joint out from under the others is exactly what would
// leak its heap allocation and leave whichever pointers referenced it
// dangling, so that operation just isn't part of the API.
//
// Move-only for the same reason Mesh/CollisionBox are elsewhere in this
// project: Robot is the sole owner, so a copy would double-delete and a
// move just needs to null out the source.
//
// Nothing derived from a Robot may outlive it: callers (e.g. buildRobot() in
// urdf_to_bullet.h) must not retain any Link*/Joint* past the Robot's
// lifetime.
class Robot {
public:
    std::string name;

    Robot() = default;
    ~Robot() { clear(); }

    Robot(const Robot&)            = delete;
    Robot& operator=(const Robot&) = delete;

    Robot(Robot&& other) noexcept
        : name(std::move(other.name)), links_(std::move(other.links_)),
          joints_(std::move(other.joints_)), root_(other.root_)
    {
        other.root_ = nullptr;
    }

    Robot& operator=(Robot&& other) noexcept
    {
        if (this != &other) {
            clear();
            name    = std::move(other.name);
            links_  = std::move(other.links_);
            joints_ = std::move(other.joints_);
            root_   = other.root_;
            other.root_ = nullptr;
        }
        return *this;
    }

    // Takes ownership of `link`; returns a stable pointer into this Robot.
    Link* addLink(Link link)
    {
        links_.push_back(new Link(std::move(link)));
        return links_.back();
    }

    // Takes ownership of `joint`. joint.parentLink/childLink must already
    // point at links this Robot returned from addLink(). Wires the edge
    // into both endpoints' Link::joints lists and marks the child as having
    // a parent.
    Joint* addJoint(Joint joint)
    {
        Joint* j = new Joint(std::move(joint));
        joints_.push_back(j);
        j->parentLink->joints.push_back(j);
        j->childLink->joints.push_back(j);
        j->childLink->hasParentJoint = true;
        return j;
    }

    // Finds and caches the link that's nobody's child. Call once after all
    // addLink()/addJoint() calls are done. Throws if there isn't one (every
    // link has a parent joint, so there must be a cycle); if more than one
    // link qualifies, the first in addLink() order is used.
    void finalize()
    {
        for (Link* link : links_) {
            if (!link->hasParentJoint) {
                root_ = link;
                break;
            }
        }
        if (!root_)
            throw std::runtime_error("urdf: no root link found (every link is some joint's child, is there a cycle?)");
    }

    const std::vector<Link*>&  links()  const { return links_; }
    const std::vector<Joint*>& joints() const { return joints_; }
    Link*                      root()   const { return root_; }

private:
    std::vector<Link*>  links_;
    std::vector<Joint*> joints_;
    Link*                root_ = nullptr;

    void clear()
    {
        for (Joint* j : joints_) delete j;
        for (Link* l : links_) delete l;
        joints_.clear();
        links_.clear();
        root_ = nullptr;
    }
};

} // namespace urdf
