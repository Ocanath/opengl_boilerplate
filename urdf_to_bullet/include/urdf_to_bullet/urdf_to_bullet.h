#pragma once
#include <string>
#include <utility>
#include <vector>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include "urdf_to_bullet/urdf_model.h"

class btDiscreteDynamicsWorld;
class btRigidBody;
class btCollisionShape;
class btTypedConstraint;

namespace urdf {

// Pose/geometry/joint -> Bullet building blocks, exposed (rather than kept
// file-local) so other build orchestrations can assemble bodies/constraints
// without going through buildRobot()'s one-rigid-body-per-link assembly.
// DynamicRobot uses these to build one rigid body per Fixed-joint-connected
// group of links instead.
btTransform toBtTransform(const Pose& pose);

// A constraint frame whose local axis (Z for hinge, X for slider — the axes
// btHingeConstraint/btSliderConstraint rotate/translate about) points along
// `axis`.
btTransform hingeFrame(const btVector3& axis);
btTransform sliderFrame(const btVector3& axis);

// Builds one btCollisionShape for a single <collision> primitive (box/
// cylinder/sphere) and appends it to outShapes for later cleanup. Throws
// std::runtime_error for Mesh geometry (urdf_parser rejects mesh <collision>
// before this would ever be reached).
btCollisionShape* buildPrimitiveShape(const Geometry& geom, std::vector<btCollisionShape*>& outShapes);

// Maps a Joint to the matching btTypedConstraint (Fixed/Revolute/Continuous/
// Prismatic -> btFixedConstraint/btHingeConstraint/.../btSliderConstraint),
// applying <limit> where applicable. Caller owns the returned constraint.
btTypedConstraint* makeJointConstraint(const Joint& joint,
                                        btRigidBody& bodyA, btRigidBody& bodyB,
                                        const btTransform& frameInA, const btTransform& frameInB);

// The geometric volume URDF describes for a single <collision> primitive,
// independent of whatever Bullet shape it became — for the same
// density-based mass fallback buildRobot()'s collisionDensity parameter
// uses (mass = density * volume when a link has no <inertial>). Exposed so
// other build orchestrations can sum it over however they group links (e.g.
// DynamicRobot sums it per merged weld-group rather than per individual link).
double geometryVolume(const Geometry& geom);

// A <visual> element, re-based from the link frame onto its rigid body's
// actual origin (the link's center of mass) so a renderer can place it with
// a single composition: body->getWorldTransform() * localTransform.
struct VisualInstance {
    btRigidBody* body = nullptr;
    Geometry     geometry;
    Material     material;
    btTransform  localTransform;
};

// A <collision> primitive, re-based the same way as VisualInstance — for a
// debug renderer that wants to draw exactly the shapes physics is using.
struct CollisionInstance {
    btRigidBody* body = nullptr;
    Geometry     geometry;
    btTransform  localTransform;
};

// Everything buildRobot() allocated and added to the world. The caller owns
// this: keep it alive for as long as the robot should exist in the world,
// and pass it to destroyBuildResult() to tear it back down.
struct BuildResult {
    // Name -> body, in the order bodies were created. Robots are small, so a
    // linear scan (see findBody() below) is simpler than a lookup table.
    std::vector<std::pair<std::string, btRigidBody*>> bodiesByLinkName;
    // Joint name -> the constraint built for it. Only non-Fixed joints get an
    // entry (a Fixed joint's constraint, if any, has nothing to drive — same
    // reasoning DynamicRobot uses to weld Fixed joints away entirely).
    std::vector<std::pair<std::string, btTypedConstraint*>> constraintsByJointName;
    std::vector<btRigidBody*>       bodies;
    std::vector<btCollisionShape*>  shapes;
    std::vector<btTypedConstraint*> constraints;
    std::vector<VisualInstance>     visuals;
    std::vector<CollisionInstance>  collisions;
};

// Linear scan for the body created for the link named `linkName`. Returns
// nullptr if there's no such link.
btRigidBody* findBody(const BuildResult& result, const std::string& linkName);

// Linear scan for the constraint built for the joint named `jointName`.
// Returns nullptr if there's no such joint (or it was Fixed and never got a
// name entry — see BuildResult::constraintsByJointName).
btTypedConstraint* findConstraint(const BuildResult& result, const std::string& jointName);

// Builds one btRigidBody per link (added to `world`) and one btTypedConstraint
// per joint (added to `world`), starting from robot.root() and placing it at
// rootTransform.
//
// Collision shapes are built only from box/cylinder/sphere <collision>
// primitives (a link with none gets an empty shape, still a valid rigid
// body). Fixed/revolute/continuous/prismatic joints map to
// btFixedConstraint/btHingeConstraint/btSliderConstraint respectively; a
// revolute joint's <limit> becomes a hinge angle limit, continuous joints
// are left free, and a prismatic joint's rotational freedom is locked so
// only its <axis> translation is free.
//
// Bodies behave like any other rigid body already in the world (subject to
// world gravity, ray casts, ability effects, etc.) — nothing here treats
// them specially.
//
// A link's mass/inertia come from its <inertial> when present. When it's
// not: mass defaults to 0 (static — a link with no <inertial> is simply
// immovable, same as Bullet's own convention for mass 0) unless
// collisionDensity > 0, in which case mass = collisionDensity * (summed
// volume of that link's <collision> primitives). Whenever a link ends up
// with mass > 0 and still no usable inertia (none given, or an explicit
// <inertial> whose ixx/iyy/izz are all exactly zero), it's left at zero
// inertia — translates normally but never rotates — unless
// calculateCollisionInertia is true, in which case it's derived from the
// link's collision shape instead.
//
// Does not retain any Link*/Joint* from `robot` past this call; `robot` may
// be destroyed immediately after buildRobot() returns.
BuildResult buildRobot(const Robot& robot,
                       btDiscreteDynamicsWorld* world,
                       const btTransform& rootTransform = btTransform::getIdentity(),
                       bool calculateCollisionInertia = false,
                       double collisionDensity = 0.0);

// Removes every body/constraint in `result` from `world` and deletes them.
void destroyBuildResult(BuildResult& result, btDiscreteDynamicsWorld* world);

} // namespace urdf
