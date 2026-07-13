#include "urdf_to_bullet/urdf_to_bullet.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <cmath>
#include <stack>
#include <stdexcept>

namespace urdf {
namespace {

constexpr double kPi = 3.14159265358979323846;

btTransform toBtTransform(const Pose& pose)
{
    btQuaternion rotation =
        btQuaternion(btVector3(0, 0, 1), (btScalar)pose.rpy.z) *
        btQuaternion(btVector3(0, 1, 0), (btScalar)pose.rpy.y) *
        btQuaternion(btVector3(1, 0, 0), (btScalar)pose.rpy.x);

    btTransform t;
    t.setIdentity();
    t.setRotation(rotation);
    t.setOrigin(btVector3((btScalar)pose.xyz.x, (btScalar)pose.xyz.y, (btScalar)pose.xyz.z));
    return t;
}

// A constraint frame whose local `axisColumn` axis (0 = X, 2 = Z) points
// along `axis`. Used because btHingeConstraint rotates about local Z and
// btSliderConstraint translates along local X.
btTransform axisAlignedFrame(const btVector3& axis, int axisColumn)
{
    btVector3 a = axis.length2() > 1e-12 ? axis.normalized() : btVector3(0, 0, 1);
    btVector3 reference = std::abs(a.dot(btVector3(0, 0, 1))) < 0.99 ? btVector3(0, 0, 1) : btVector3(1, 0, 0);
    btVector3 b = a.cross(reference).normalized();
    btVector3 c = a.cross(b).normalized();

    btMatrix3x3 basis;
    if (axisColumn == 0) {
        // columns: a, b, c
        basis.setValue(a.x(), b.x(), c.x(),
                        a.y(), b.y(), c.y(),
                        a.z(), b.z(), c.z());
    } else {
        // columns: b, c, a
        basis.setValue(b.x(), c.x(), a.x(),
                        b.y(), c.y(), a.y(),
                        b.z(), c.z(), a.z());
    }

    btTransform t;
    t.setIdentity();
    t.setBasis(basis);
    return t;
}

btTransform hingeFrame(const btVector3& axis) { return axisAlignedFrame(axis, 2); }
btTransform sliderFrame(const btVector3& axis) { return axisAlignedFrame(axis, 0); }

btCollisionShape* buildPrimitiveShape(const Geometry& geom, std::vector<btCollisionShape*>& outShapes)
{
    btCollisionShape* shape = nullptr;
    switch (geom.type) {
        case GeometryType::Box:
            shape = new btBoxShape(btVector3((btScalar)(geom.boxSize.x * 0.5),
                                              (btScalar)(geom.boxSize.y * 0.5),
                                              (btScalar)(geom.boxSize.z * 0.5)));
            break;
        case GeometryType::Cylinder:
            shape = new btCylinderShapeZ(btVector3((btScalar)geom.cylinderRadius,
                                                     (btScalar)geom.cylinderRadius,
                                                     (btScalar)(geom.cylinderLength * 0.5)));
            break;
        case GeometryType::Sphere:
            shape = new btSphereShape((btScalar)geom.sphereRadius);
            break;
        case GeometryType::Mesh:
            // urdf_parser rejects mesh <collision> geometry before this is ever reached.
            throw std::runtime_error("urdf_to_bullet: mesh collision geometry is not supported");
    }
    outShapes.push_back(shape);
    return shape;
}

// One shape per link: an (unshared) btCompoundShape holding every <collision>
// primitive at its local offset from the body's origin (the link's center of
// mass), or a btEmptyShape if the link has no collision geometry at all —
// still a valid rigid body, just one that nothing can hit.
btCollisionShape* buildLinkShape(const Link& link,
                                  const btTransform& comLocalInverse,
                                  std::vector<btCollisionShape*>& outShapes)
{
    if (link.collisions.empty()) {
        auto* empty = new btEmptyShape();
        outShapes.push_back(empty);
        return empty;
    }

    auto* compound = new btCompoundShape();
    outShapes.push_back(compound);
    for (const Collision& c : link.collisions) {
        btCollisionShape* child = buildPrimitiveShape(c.geometry, outShapes);
        compound->addChildShape(comLocalInverse * toBtTransform(c.origin), child);
    }
    return compound;
}

// For the collisionDensity fallback: the geometric volume URDF describes,
// independent of whatever Bullet shape it became.
double geometryVolume(const Geometry& geom)
{
    switch (geom.type) {
        case GeometryType::Box:
            return geom.boxSize.x * geom.boxSize.y * geom.boxSize.z;
        case GeometryType::Cylinder:
            return kPi * geom.cylinderRadius * geom.cylinderRadius * geom.cylinderLength;
        case GeometryType::Sphere:
            return (4.0 / 3.0) * kPi * geom.sphereRadius * geom.sphereRadius * geom.sphereRadius;
        case GeometryType::Mesh:
            return 0.0; // urdf_parser rejects mesh <collision>; unreachable in practice
    }
    return 0.0;
}

double linkCollisionVolume(const Link& link)
{
    double total = 0.0;
    for (const Collision& c : link.collisions)
        total += geometryVolume(c.geometry);
    return total;
}

// One entry on the DFS stack: a link that's been reached and is ready to
// become a body, plus (except for the root) the edge that reached it and
// the nearest real (non-welded — see isWeldedAway below) ancestor body for
// that edge's other end.
struct PendingLink {
    const Link*  link;
    btTransform  linkFrame;                // this link's own frame, in world space
    const Joint* incomingJoint = nullptr;  // edge that reached this link; null for the root
    btRigidBody* parentBody = nullptr;     // nearest real ancestor body; null for the root
};

// A link with no <collision>, <visual>, or <inertial> at all is a pure
// transform pass-through in the tree — common for e.g. a "bone" link that
// only exists to carry a fixed offset between two real parts. Giving it its
// own rigid body is actively wrong once any neighboring link has mass:
// mass-0 in Bullet means immovable, so a massless link wedged between two
// dynamic links via fixed joints would anchor part of the robot to a fixed
// point in space while the rest of it tries to move/fall — an unsatisfiable
// constraint every solver step, which explodes.
//
// So such a link gets no body at all: its joint's transform is folded into
// whatever real body eventually appears further down the chain (the DFS's
// existing linkFrame accumulation already does this automatically). This
// only works when the joint feeding into it is fixed — a revolute/
// continuous/prismatic joint is an actual runtime degree of freedom and
// can't be folded into a static transform, so a contentless link reached
// that way still gets a normal (if massless) body.
bool isWeldedAway(const Link& link, const Joint* incomingJoint)
{
    return incomingJoint
        && incomingJoint->type == JointType::Fixed
        && link.collisions.empty()
        && link.visuals.empty()
        && !link.inertial.present;
}

btTypedConstraint* makeJointConstraint(const Joint& joint,
                                        btRigidBody& bodyA, btRigidBody& bodyB,
                                        const btTransform& frameInA, const btTransform& frameInB)
{
    switch (joint.type) {
        case JointType::Fixed:
            return new btFixedConstraint(bodyA, bodyB, frameInA, frameInB);

        case JointType::Revolute:
        case JointType::Continuous: {
            btVector3 axis((btScalar)joint.axis.x, (btScalar)joint.axis.y, (btScalar)joint.axis.z);
            btTransform align = hingeFrame(axis);
            auto* hinge = new btHingeConstraint(bodyA, bodyB, frameInA * align, frameInB * align);
            if (joint.type == JointType::Revolute && joint.limit.present)
                hinge->setLimit((btScalar)joint.limit.lower, (btScalar)joint.limit.upper);
            return hinge;
        }

        case JointType::Prismatic: {
            btVector3 axis((btScalar)joint.axis.x, (btScalar)joint.axis.y, (btScalar)joint.axis.z);
            btTransform align = sliderFrame(axis);
            auto* slider = new btSliderConstraint(bodyA, bodyB, frameInA * align, frameInB * align, true);
            // A prismatic joint only translates: lock the slider's free rotation about its axis.
            slider->setLowerAngLimit(0.0);
            slider->setUpperAngLimit(0.0);
            if (joint.limit.present) {
                slider->setLowerLinLimit((btScalar)joint.limit.lower);
                slider->setUpperLinLimit((btScalar)joint.limit.upper);
            }
            return slider;
        }
    }
    return nullptr; // unreachable: every JointType is handled above
}

} // namespace

btRigidBody* findBody(const BuildResult& result, const std::string& linkName)
{
    for (const auto& entry : result.bodiesByLinkName)
        if (entry.first == linkName) return entry.second;
    return nullptr;
}

BuildResult buildRobot(const Robot& robot, btDiscreteDynamicsWorld* world, const btTransform& rootTransform,
                        bool calculateCollisionInertia, double collisionDensity)
{
    BuildResult result;

    // Iterative depth-first walk of the link/joint tree, same shape as
    // kin_dfs_dq() in embedded-kinematics/src/kinematics_dual_quat.c: push
    // the root, and each time a link is popped, walk its `joints` list and
    // push the child of every joint where this link is the parent
    // (joint->parentLink == link — the "is this my child" filter). A link's
    // rigid body, and the constraint for the edge that reached it, are both
    // built at pop time: the parent body already exists by then, since it
    // was built when its own entry was popped earlier.
    std::stack<PendingLink> stack;
    stack.push({ robot.root(), rootTransform });

    while (!stack.empty()) {
        PendingLink cur = stack.top();
        stack.pop();

        if (isWeldedAway(*cur.link, cur.incomingJoint)) {
            // No body for this link: fold its joint into whatever real body
            // its own children eventually reach. cur.linkFrame already is
            // this link's fully composed world frame, so the children's
            // linkFrame composition (below) needs no special-casing — only
            // parentBody must skip over this link, forwarding the same real
            // ancestor its own incomingJoint edge came from.
            for (Joint* joint : cur.link->joints) {
                if (joint->parentLink != cur.link) continue;
                stack.push({ joint->childLink, cur.linkFrame * toBtTransform(joint->origin), joint, cur.parentBody });
            }
            continue;
        }

        btTransform comLocal = cur.link->inertial.present ? toBtTransform(cur.link->inertial.origin) : btTransform::getIdentity();
        btTransform comLocalInverse = comLocal.inverse();
        btTransform bodyWorldTransform = cur.linkFrame * comLocal;

        btCollisionShape* shape = buildLinkShape(*cur.link, comLocalInverse, result.shapes);

        double mass;
        btVector3 localInertia(0, 0, 0);
        if (cur.link->inertial.present) {
            mass = cur.link->inertial.mass;
            localInertia = btVector3((btScalar)cur.link->inertial.ixx, (btScalar)cur.link->inertial.iyy, (btScalar)cur.link->inertial.izz);
        } else {
            mass = collisionDensity > 0.0 ? collisionDensity * linkCollisionVolume(*cur.link) : 0.0;
        }
        if (mass > 0.0 && localInertia.fuzzyZero() && calculateCollisionInertia)
            shape->calculateLocalInertia((btScalar)mass, localInertia);

        auto* motionState = new btDefaultMotionState(bodyWorldTransform);
        btRigidBody::btRigidBodyConstructionInfo ci((btScalar)mass, motionState, shape, localInertia);
        auto* body = new btRigidBody(ci);
        world->addRigidBody(body);

        result.bodies.push_back(body);
        result.bodiesByLinkName.push_back({ cur.link->name, body });

        for (const Visual& v : cur.link->visuals) {
            VisualInstance vi;
            vi.body = body;
            vi.geometry = v.geometry;
            vi.material = v.material;
            vi.localTransform = comLocalInverse * toBtTransform(v.origin);
            result.visuals.push_back(std::move(vi));
        }

        for (const Collision& c : cur.link->collisions) {
            CollisionInstance ci;
            ci.body = body;
            ci.geometry = c.geometry;
            ci.localTransform = comLocalInverse * toBtTransform(c.origin);
            result.collisions.push_back(std::move(ci));
        }

        // The edge that reached this link connects two now-existing bodies —
        // cur.parentBody may be several welded-away links back up the chain,
        // so its frame is derived from that body's actual world transform
        // (already set from its motion state) rather than re-deriving it
        // through any single joint's <origin>, which by itself would only
        // account for the last hop.
        if (cur.incomingJoint) {
            btTransform frameInA = cur.parentBody->getWorldTransform().inverse() * cur.linkFrame;
            btTransform frameInB = comLocalInverse;
            btTypedConstraint* constraint =
                makeJointConstraint(*cur.incomingJoint, *cur.parentBody, *body, frameInA, frameInB);
            world->addConstraint(constraint, /*disableCollisionsBetweenLinkedBodies=*/true);
            result.constraints.push_back(constraint);
        }

        for (Joint* joint : cur.link->joints) {
            if (joint->parentLink != cur.link) continue; // only follow edges down to children
            stack.push({ joint->childLink, cur.linkFrame * toBtTransform(joint->origin), joint, body });
        }
    }

    return result;
}

void destroyBuildResult(BuildResult& result, btDiscreteDynamicsWorld* world)
{
    for (btTypedConstraint* c : result.constraints) {
        world->removeConstraint(c);
        delete c;
    }
    for (btRigidBody* b : result.bodies) {
        world->removeRigidBody(b);
        delete b->getMotionState();
        delete b;
    }
    for (btCollisionShape* s : result.shapes)
        delete s;

    result = BuildResult{};
}

} // namespace urdf
