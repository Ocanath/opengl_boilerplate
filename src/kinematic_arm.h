#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "collision_box.h"

class btDiscreteDynamicsWorld;
class btHingeConstraint;
class Model;
class Shader;

// N-link planar arm in the XY plane.
// Links are dynamic rigid bodies connected by hinge constraints (Z axis).
// Joint motors are driven by a proportional controller toward target angles.
// Joints with no corresponding theta entry are left free (passive/floppy).
class KinematicArm {
public:
    struct LinkDef {
        float     length;
        float     width  = 0.3f;
        float     depth  = 0.3f;
        float     mass   = 1.f;
        glm::vec3 color  = {0.8f, 0.4f, 0.1f};
    };

    // base: world position of the root joint.
    // motorGain: proportional gain (rad/s per rad of error); lower = floppier.
    // maxImpulse: max motor impulse per step; lower = weaker joints.
    KinematicArm(btDiscreteDynamicsWorld* world,
                 Model*                   cubeModel,
                 const std::vector<LinkDef>& defs,
                 glm::vec3                base       = {0.f, 0.f, 0.5f},
                 float                    motorGain  = 8.f,
                 float                    maxImpulse = 5.f);
    ~KinematicArm();

    KinematicArm(const KinematicArm&)            = delete;
    KinematicArm& operator=(const KinematicArm&) = delete;
    KinematicArm(KinematicArm&&)                 = delete;

    // theta[i]: target angle for joint i relative to joint i-1 (radians).
    // Joints beyond thetas.size() have their motors disabled (free).
    void setThetas(const std::vector<float>& thetas);

    void draw(Shader& shader) const;

private:
    btDiscreteDynamicsWorld*         world_     = nullptr;
    std::vector<LinkDef>             defs_;
    std::vector<CollisionBox>        links_;
    std::vector<btHingeConstraint*>  hinges_;   // N-1 entries for N links
    float                            motorGain_;
    float                            maxImpulse_;
};
