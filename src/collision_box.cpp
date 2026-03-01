#include "collision_box.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>

CollisionBox::CollisionBox(btDiscreteDynamicsWorld* world,
                           Model*      model,
                           glm::vec3   halfExtents,
                           glm::vec3   position,
                           glm::vec3   visualScale,
                           glm::vec3   color,
                           float       mass,
                           glm::quat   rotation,
                           bool        kinematic)
    : world_(world), model_(model), visualScale_(visualScale), color_(color)
{
    shape_ = new btBoxShape({ halfExtents.x, halfExtents.y, halfExtents.z });

    btTransform t;
    t.setIdentity();
    t.setOrigin({ position.x, position.y, position.z });
    t.setRotation({ rotation.x, rotation.y, rotation.z, rotation.w }); // btQuat(x,y,z,w)

    motion_ = new btDefaultMotionState(t);

    btVector3 inertia(0, 0, 0);
    if (mass > 0.f)
        shape_->calculateLocalInertia(mass, inertia);

    btRigidBody::btRigidBodyConstructionInfo ci(mass, motion_, shape_, inertia);
    body_ = new btRigidBody(ci);

    if (kinematic) {
        body_->setCollisionFlags(body_->getCollisionFlags() |
                                 btCollisionObject::CF_KINEMATIC_OBJECT);
        body_->setActivationState(DISABLE_DEACTIVATION);
    }

    world_->addRigidBody(body_);
}

CollisionBox::~CollisionBox()
{
    if (body_ && world_) {
        world_->removeRigidBody(body_);
        delete body_;
        body_ = nullptr;
    }
    delete motion_;
    motion_ = nullptr;
    delete shape_;
    shape_  = nullptr;
    world_  = nullptr;
}

CollisionBox::CollisionBox(CollisionBox&& o) noexcept
    : world_(o.world_), shape_(o.shape_), body_(o.body_), motion_(o.motion_),
      model_(o.model_), visualScale_(o.visualScale_), color_(o.color_)
{
    o.world_  = nullptr;
    o.shape_  = nullptr;
    o.body_   = nullptr;
    o.motion_ = nullptr;
}

CollisionBox& CollisionBox::operator=(CollisionBox&& o) noexcept
{
    if (this != &o) {
        if (body_ && world_) {
            world_->removeRigidBody(body_);
            delete body_;
        }
        delete motion_;
        delete shape_;

        world_       = o.world_;
        shape_       = o.shape_;
        body_        = o.body_;
        motion_      = o.motion_;
        model_       = o.model_;
        visualScale_ = o.visualScale_;
        color_       = o.color_;

        o.world_  = nullptr;
        o.shape_  = nullptr;
        o.body_   = nullptr;
        o.motion_ = nullptr;
    }
    return *this;
}

void CollisionBox::draw(Shader& shader) const
{
    if (!model_) return;
    shader.setMat4("model",       getModelMatrix());
    shader.setVec3("objectColor", color_);
    model_->draw(shader);
}

void CollisionBox::syncKinematic(const glm::vec3& pos)
{
    if (!body_) return;
    btTransform t;
    t.setIdentity();
    t.setOrigin({ pos.x, pos.y, pos.z });
    body_->getMotionState()->setWorldTransform(t);
    body_->setWorldTransform(t);
}

glm::vec3 CollisionBox::getPosition() const
{
    btTransform t;
    body_->getMotionState()->getWorldTransform(t);
    btVector3 p = t.getOrigin();
    return { p.x(), p.y(), p.z() };
}

glm::mat4 CollisionBox::getModelMatrix() const
{
    btTransform t;
    body_->getMotionState()->getWorldTransform(t);
    btVector3 p = t.getOrigin();
    glm::mat4 m = glm::translate(glm::mat4(1.f), { p.x(), p.y(), p.z() });
    return glm::scale(m, visualScale_);
}
