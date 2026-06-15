#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "collision_box.h"

class btDiscreteDynamicsWorld;
class Model;
class Shader;

// Planar N-DOF arm rotating in the XY plane (around Z).
// Each link is a kinematic Bullet rigid body driven by forward kinematics from
// encoder thetas. Designed to swap in custom meshes later — only requires the
// cube Model* for the box placeholder.
class KinematicArm {
public:
    struct LinkDef {
        float     length;                        // along local X (arm direction)
        float     width  = 0.3f;                 // local Y
        float     depth  = 0.3f;                 // local Z
        glm::vec3 color  = {0.8f, 0.4f, 0.1f};
    };

    // base: world position of the root joint.
    KinematicArm(btDiscreteDynamicsWorld* world,
                 Model*                   cubeModel,
                 const std::vector<LinkDef>& defs,
                 glm::vec3                base = {0.f, 0.f, 1.f});

    // theta[0]: rotation of link 0 from world +X (radians).
    // theta[i]: rotation of link i relative to link i-1 direction.
    // If fewer thetas than links are supplied the remaining joints stay fixed.
    void setThetas(const std::vector<float>& thetas);

    void draw(Shader& shader) const;

    int linkCount() const { return (int)links_.size(); }

private:
    glm::vec3                base_;
    std::vector<LinkDef>     defs_;
    std::vector<CollisionBox> links_;
    std::vector<float>        cumAngles_; // cached for getModelMatrix correctness
};
