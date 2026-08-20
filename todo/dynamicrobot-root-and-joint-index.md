# DynamicRobot: hand-of-god root control + indexed joint motors

Goal: expose enough on `DynamicRobot` that a future input-mapping layer
(WASD embodiment, keybindings, etc.) can drive the robot's root through the
world and drive individual joints, without that layer touching Bullet
directly.

Two independent features, can be done in either order.

## 1. Hand-of-god root control

Move the whole robot (root body of the kinematic tree — the supernode
containing `root_`) around physically via Bullet velocity, with an on/off
toggle so it can be released back to normal physics.

- [ ] Store the root body: `btRigidBody * rootBody_ = nullptr;`, set to
      `supernodes[0].body` once it's built in `buildBulletRobot()`
      (`supernodes[0]` always contains `root_` — never reassigned).
- [ ] `void setHandOfGod(bool enabled)` — mirrors `Camera::setGravity()`:
      zero `rootBody_`'s gravity when enabling (so it holds position under
      velocity commands instead of falling), restore `world_->getGravity()`
      when disabling. Also `activate(true)` so a sleeping body wakes up.
- [ ] `bool isHandOfGod() const`
- [ ] `void setRootVelocity(const btVector3 & linear, const btVector3 & angular)`
      — direct wrapper around `rootBody_->setLinearVelocity()` /
      `setAngularVelocity()` (world frame, rad/s for angular). Not gated on
      `setHandOfGod()` — it's just physics, usable any time. Calls
      `activate(true)`.

Open question / design decision to confirm: on `setHandOfGod(false)`, do we
leave whatever velocity was last commanded (simplest, momentum carries into
normal physics), or zero it automatically? Leaning toward **leave as-is**
(caller can `setRootVelocity(0,0)` first for a hard stop) — confirm before
implementing.

## 2. Joint motors exposed by index (URDF document order) — DONE

Today `setJointVelocity`/`setJointTargetAngle` only take a joint name and do
a linear string-scan over `buildResult_.constraintsByJointName` every call.
For a hot per-frame loop (one physical encoder driving one joint every
tick), an index into a vector avoids that scan.

- [x] After `buildBulletRobot()`'s main supernode loop, build:
      - `std::vector<btTypedConstraint*> jointMotorsByIndex_;`
      - `std::vector<std::string> jointNamesByIndex_;`
      by walking `joints_` in order (this vector is already in exact URDF
      `<joint>` document order — `addJoint()` appends in parse order) and
      skipping `JointType::Fixed` (welded away, never becomes its own
      constraint). Look up each surviving joint's constraint via the
      existing `findConstraint(buildResult_, joint->name)`.
- [x] `size_t getJointCount() const`
- [x] `const std::string & getJointName(size_t index) const` (bounds-checked,
      warns and returns a static empty string on out-of-range)
- [x] `void setJointVelocity(size_t index, double velocity, double maxImpulse)`
- [x] `void setJointTargetAngle(size_t index, double targetAngle, double dt, double maxImpulse)`
- [x] Refactor: factor the existing `findHingeJoint(BuildResult&, name, caller)`
      static helper into two pieces so both the name-keyed and index-keyed
      paths share one "validate + cast to btHingeConstraint*" implementation
      instead of duplicating the type-check/warning logic.

## Out of scope for this pass

- The actual WASD/mouse → `setRootVelocity`/joint-index keybinding layer
  (decided: WASD moves the floating root/torso, mouse drives a tagged
  look-joint if one exists — but the input-mapping code itself is a
  separate step, not part of this DynamicRobot-side change).
- Any `<sensor>`/camera-tag URDF convention for FPV camera switching.
- PD position controller replacing the crude `setMotorTarget` wrapper in
  `setJointTargetAngle` (discussed, deferred until after the robot build-out
  is further along).
