#include "urdf_to_bullet/urdf_to_bullet.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace urdf {
namespace {

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

struct LinkRuntime {
    btRigidBody* body = nullptr;
    btTransform  comLocalInverse; // link frame -> body (center-of-mass) frame
};

} // namespace

BuildResult buildRobot(const Robot& robot, btDiscreteDynamicsWorld* world, const btTransform& rootTransform)
{
    BuildResult result;

    std::unordered_map<std::string, const Link*> linkByName;
    for (const Link& link : robot.links) linkByName[link.name] = &link;

    std::unordered_map<std::string, std::vector<const Joint*>> childJointsByParent;
    std::unordered_set<std::string> childLinkNames;
    for (const Joint& joint : robot.joints) {
        childJointsByParent[joint.parentLink].push_back(&joint);
        childLinkNames.insert(joint.childLink);
    }

    std::unordered_map<std::string, LinkRuntime> runtime;

    // Pass 1: walk the tree from every root (a link that's never a joint's
    // child) to place and create each link's rigid body.
    std::vector<std::pair<const Link*, btTransform>> pending;
    for (const Link& link : robot.links)
        if (!childLinkNames.count(link.name))
            pending.push_back({ &link, rootTransform });

    while (!pending.empty()) {
        auto [link, linkFrame] = pending.back();
        pending.pop_back();

        btTransform comLocal = link->inertial.present ? toBtTransform(link->inertial.origin) : btTransform::getIdentity();
        btTransform comLocalInverse = comLocal.inverse();
        btTransform bodyWorldTransform = linkFrame * comLocal;

        btCollisionShape* shape = buildLinkShape(*link, comLocalInverse, result.shapes);

        double mass = link->inertial.present ? link->inertial.mass : 0.0;
        btVector3 localInertia(0, 0, 0);
        if (mass > 0.0) {
            localInertia = btVector3((btScalar)link->inertial.ixx, (btScalar)link->inertial.iyy, (btScalar)link->inertial.izz);
            if (localInertia.fuzzyZero())
                shape->calculateLocalInertia((btScalar)mass, localInertia);
        }

        auto* motionState = new btDefaultMotionState(bodyWorldTransform);
        btRigidBody::btRigidBodyConstructionInfo ci((btScalar)mass, motionState, shape, localInertia);
        auto* body = new btRigidBody(ci);
        world->addRigidBody(body);

        result.bodies.push_back(body);
        result.bodiesByLinkName[link->name] = body;
        runtime[link->name] = { body, comLocalInverse };

        for (const Visual& v : link->visuals) {
            VisualInstance vi;
            vi.body = body;
            vi.geometry = v.geometry;
            vi.material = v.material;
            vi.localTransform = comLocalInverse * toBtTransform(v.origin);
            result.visuals.push_back(std::move(vi));
        }

        auto childrenIt = childJointsByParent.find(link->name);
        if (childrenIt != childJointsByParent.end()) {
            for (const Joint* joint : childrenIt->second) {
                auto childLinkIt = linkByName.find(joint->childLink);
                if (childLinkIt == linkByName.end())
                    throw std::runtime_error("urdf_to_bullet: joint \"" + joint->name + "\" references unknown child link \"" + joint->childLink + "\"");
                pending.push_back({ childLinkIt->second, linkFrame * toBtTransform(joint->origin) });
            }
        }
    }

    // Pass 2: every body now exists, so build joint constraints by name.
    for (const Joint& joint : robot.joints) {
        auto parentIt = runtime.find(joint.parentLink);
        auto childIt  = runtime.find(joint.childLink);
        if (parentIt == runtime.end() || childIt == runtime.end())
            throw std::runtime_error("urdf_to_bullet: joint \"" + joint.name + "\" references a link with no body");

        btRigidBody& bodyA = *parentIt->second.body;
        btRigidBody& bodyB = *childIt->second.body;

        // The joint frame coincides with the child link's frame, so from the
        // parent's side it's offset by the joint's <origin>; from the
        // child's side there's no additional offset.
        btTransform frameInA = parentIt->second.comLocalInverse * toBtTransform(joint.origin);
        btTransform frameInB = childIt->second.comLocalInverse;

        btTypedConstraint* constraint = nullptr;
        switch (joint.type) {
            case JointType::Fixed:
                constraint = new btFixedConstraint(bodyA, bodyB, frameInA, frameInB);
                break;

            case JointType::Revolute:
            case JointType::Continuous: {
                btVector3 axis((btScalar)joint.axis.x, (btScalar)joint.axis.y, (btScalar)joint.axis.z);
                btTransform align = hingeFrame(axis);
                auto* hinge = new btHingeConstraint(bodyA, bodyB, frameInA * align, frameInB * align);
                if (joint.type == JointType::Revolute && joint.limit.present)
                    hinge->setLimit((btScalar)joint.limit.lower, (btScalar)joint.limit.upper);
                constraint = hinge;
                break;
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
                constraint = slider;
                break;
            }
        }

        world->addConstraint(constraint, /*disableCollisionsBetweenLinkedBodies=*/true);
        result.constraints.push_back(constraint);
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
