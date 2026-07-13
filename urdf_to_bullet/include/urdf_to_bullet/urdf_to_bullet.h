#pragma once
#include <unordered_map>
#include <vector>
#include <LinearMath/btTransform.h>
#include "urdf_to_bullet/urdf_model.h"

class btDiscreteDynamicsWorld;
class btRigidBody;
class btCollisionShape;
class btTypedConstraint;

namespace urdf {

// A <visual> element, re-based from the link frame onto its rigid body's
// actual origin (the link's center of mass) so a renderer can place it with
// a single composition: body->getWorldTransform() * localTransform.
struct VisualInstance {
    btRigidBody* body = nullptr;
    Geometry     geometry;
    Material     material;
    btTransform  localTransform;
};

// Everything buildRobot() allocated and added to the world. The caller owns
// this: keep it alive for as long as the robot should exist in the world,
// and pass it to destroyBuildResult() to tear it back down.
struct BuildResult {
    std::unordered_map<std::string, btRigidBody*> bodiesByLinkName;
    std::vector<btRigidBody*>       bodies;
    std::vector<btCollisionShape*>  shapes;
    std::vector<btTypedConstraint*> constraints;
    std::vector<VisualInstance>     visuals;
};

// Builds one btRigidBody per link (added to `world`) and one btTypedConstraint
// per joint (added to `world`), placing the robot's root link(s) — links that
// are never a joint's child — at rootTransform.
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
BuildResult buildRobot(const Robot& robot,
                       btDiscreteDynamicsWorld* world,
                       const btTransform& rootTransform = btTransform::getIdentity());

// Removes every body/constraint in `result` from `world` and deletes them.
void destroyBuildResult(BuildResult& result, btDiscreteDynamicsWorld* world);

} // namespace urdf
