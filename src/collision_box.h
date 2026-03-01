#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "model.h"
#include "shader.h"

class btDiscreteDynamicsWorld;
class btBoxShape;
class btRigidBody;
class btMotionState;

class CollisionBox {
public:
    CollisionBox(btDiscreteDynamicsWorld* world,
                 Model*      model,        // non-owning ptr for rendering
                 glm::vec3   halfExtents,  // physics shape half-size
                 glm::vec3   position,
                 glm::vec3   visualScale,  // how to scale unit cube mesh
                 glm::vec3   color,
                 float       mass      = 0.f,
                 glm::quat   rotation  = glm::quat(1.f, 0.f, 0.f, 0.f),
                 bool        kinematic = false);
    ~CollisionBox();

    CollisionBox(CollisionBox&&) noexcept;
    CollisionBox& operator=(CollisionBox&&) noexcept;
    CollisionBox(const CollisionBox&)            = delete;
    CollisionBox& operator=(const CollisionBox&) = delete;

    void      draw(Shader& shader) const;
    void      syncKinematic(const glm::vec3& pos); // kinematic bodies only
    glm::vec3 getPosition()    const;
    glm::mat4 getModelMatrix() const;
    btRigidBody* getBody() const { return body_; }

private:
    btDiscreteDynamicsWorld* world_  = nullptr;
    btBoxShape*              shape_  = nullptr;
    btRigidBody*             body_   = nullptr;
    btMotionState*           motion_ = nullptr;
    Model*                   model_  = nullptr;
    glm::vec3                visualScale_;
    glm::vec3                color_;
};
