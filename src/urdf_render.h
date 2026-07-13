#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "urdf_to_bullet/urdf_to_bullet.h"

class Model;
class Shader;
class btRigidBody;

// Binds a built URDF robot (urdf::BuildResult) to drawable geometry so it
// can be drawn following its physics bodies each frame: visual meshes/
// primitives from <visual>, and separately, the box/cylinder/sphere
// primitives physics actually uses from <collision> (for confirming
// collision geometry visually — e.g. while a robot's real meshes are still
// broken/unresolved, as puppet.urdf's are right now).
//
// meshBaseDir is prepended to each <mesh filename="..."> before loading —
// urdf_to_bullet leaves that path exactly as written in the URDF, unresolved.
//
// A visual mesh that fails to load (missing file, format assimp can't read)
// logs one warning — not one per frame, not one per link reusing the same
// broken path — and that link just renders with no visual; nothing throws
// past construction.
class UrdfRender {
public:
    UrdfRender(const urdf::BuildResult& build, const std::string& meshBaseDir);

    // Call during the deferred G-buffer pass (lit, like the rest of the scene).
    void drawVisual(Shader& shader) const;

    // Call during the unlit forward pass; drawn in a fixed debug color since
    // <collision> carries no material.
    void drawCollision(Shader& shader) const;

private:
    struct Instance {
        btRigidBody* body = nullptr;
        Model*       model = nullptr;
        glm::mat4    localTransform{1.f}; // body origin -> geometry frame, scale included
        glm::vec3    color{1.f};
    };

    std::vector<Instance> visualInstances_;
    std::vector<Instance> collisionInstances_;
};
