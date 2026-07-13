#pragma once
#include <string>
#include <utility>
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
    // Name -> body, in the order bodies were created. Robots are small, so a
    // linear scan (see findBody() below) is simpler than a lookup table.
    std::vector<std::pair<std::string, btRigidBody*>> bodiesByLinkName;
    std::vector<btRigidBody*>       bodies;
    std::vector<btCollisionShape*>  shapes;
    std::vector<btTypedConstraint*> constraints;
    std::vector<VisualInstance>     visuals;
};

// Linear scan for the body created for the link named `linkName`. Returns
// nullptr if there's no such link.
btRigidBody* findBody(const BuildResult& result, const std::string& linkName);

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
// Does not retain any Link*/Joint* from `robot` past this call; `robot` may
// be destroyed immediately after buildRobot() returns.
BuildResult buildRobot(const Robot& robot,
                       btDiscreteDynamicsWorld* world,
                       const btTransform& rootTransform = btTransform::getIdentity());

// Removes every body/constraint in `result` from `world` and deletes them.
void destroyBuildResult(BuildResult& result, btDiscreteDynamicsWorld* world);

} // namespace urdf
