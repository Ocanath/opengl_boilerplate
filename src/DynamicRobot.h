#pragma once
#include <string>
#include <vector>
#include <optional>
#include <tinyxml2.h>
#include "urdf_to_bullet/urdf_model.h"
#include "urdf_to_bullet/urdf_to_bullet.h"
#include "urdf_render.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using namespace urdf;

class DynamicRobot
{
	public:
		std::string name;
		XMLDocument doc;
		// spawnPosition places root_ (and, transitively, the whole tree) in
		// the world; buildBulletRobot() reads it when constructing bodies.
		// meshBaseDir is prepended to <mesh filename="..."> paths when
		// render_ is built, same convention as UrdfRender itself.
		// collisionDensity: for a supernode member with no <inertial>, mass
		// (and, from the shape, inertia) is derived from this density times
		// that member's <collision> volume instead of defaulting to 0. 0
		// (the default) keeps such members massless/static, same as before.
		// scale: uniform dimensionless multiplier applied to every length in
		// the parsed URDF (positions, box/cylinder/sphere dimensions, mesh
		// scale) at the start of buildBulletRobot() — e.g. a URDF authored at
		// its real (tiny) scale can be blown up to match this scene's world
		// units. Mass/inertia naturally come out based on the *scaled*
		// volumes, since they're computed from this same (already-scaled)
		// geometry data.
		// jointDamping: 0 (default) leaves every hinge frictionless, same as
		// Bullet's own default. Nonzero gives every revolute/continuous
		// joint a standing motor targeting 0 velocity with this as its max
		// impulse — a bounded resistive brake, since Bullet constraints have
		// no built-in friction otherwise. setJointVelocity()/
		// setJointTargetAngle() just reconfigure that same motor for
		// deliberate control.
		DynamicRobot(const std::string & path, const btVector3 & spawnPosition = btVector3(0, 0, 0),
		             const std::string & meshBaseDir = "", double collisionDensity = 0.0, double scale = 1.0,
		             double jointDamping = 0.0);
		~DynamicRobot();
		void traverse_tree_dfs(void);	//test
		void buildBulletRobot(btDiscreteDynamicsWorld * world);

		// <visual> geometry and <collision> primitives respectively, each
		// following its supernode's rigid body. Both are no-ops until
		// buildBulletRobot() has run — that's what builds render_.
		void render(Shader & shader) const;
		void renderCollision(Shader & shader) const;

		// Hinge (revolute/continuous) joint control, looked up by <joint
		// name="...">. No-op (prints a warning) if jointName doesn't exist or
		// isn't a hinge. Both just reconfigure the same motor jointDamping
		// (if nonzero) already enabled on every hinge.
		//
		// velocity: rad/s, positive per the joint's own <axis>. maxImpulse
		// bounds how hard the motor can push per solver substep — the
		// smaller of the two limits how fast/strongly the joint can move.
		void setJointVelocity(const std::string & jointName, double velocity, double maxImpulse);

		// Convenience position control: drives toward targetAngle (radians)
		// over the next `dt` seconds — call this every physics step with the
		// simulation's fixed timestep, same contract as
		// btHingeConstraint::setMotorTarget().
		void setJointTargetAngle(const std::string & jointName, double targetAngle, double dt, double maxImpulse);

		// Same as the name-keyed overloads above, but by position in the
		// URDF's own <joint> element order (Fixed joints excluded — they're
		// welded away and never become their own constraint, so they have
		// nothing to expose here). Meant for a hot per-frame control loop
		// (e.g. one physical encoder driving one joint every tick): an
		// index lookup is O(1) into a vector, where the name-keyed overloads
		// do a linear string-compare scan every call.
		size_t getJointCount() const;
		const std::string & getJointName(size_t index) const;
		void setJointVelocity(size_t index, double velocity, double maxImpulse);
		void setJointTargetAngle(size_t index, double targetAngle, double dt, double maxImpulse);
	private:
		std::vector<Link*> links_;
		std::vector<Joint*> joints_;
		Link * root_ = nullptr;
		BuildResult buildResult_;
		btDiscreteDynamicsWorld * world_ = nullptr; // set by buildBulletRobot(); ~DynamicRobot() uses it to tear buildResult_ back out of the world
		btTransform rootTransform_ = btTransform::getIdentity();
		std::string meshBaseDir_;
		double collisionDensity_ = 0.0;
		double scale_ = 1.0;
		bool scaled_ = false; // guards against re-applying scale_ if buildBulletRobot() is ever called twice
		double jointDamping_ = 0.0;
		std::optional<UrdfRender> render_;
		std::vector<btTypedConstraint*> jointMotorsByIndex_; // URDF <joint> document order, Fixed joints excluded
		std::vector<std::string> jointNamesByIndex_;         // parallel to jointMotorsByIndex_

		void addLink(const XMLElement * link);
		void addJoint(const XMLElement * xml_joint);
		void assignRoot(void);
		void assignJoints(void);
		void applyScale(double scale);

};



