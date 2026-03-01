#include "camera.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <btBulletDynamicsCommon.h>
#include <algorithm>
#include <cmath>

Camera::Camera(btDiscreteDynamicsWorld* world, glm::vec3 startPos)
    : world_(world)
{
    // Capsule shape: radius 0.3, height 1.8
    shape_ = new btCapsuleShape(0.3f, 1.8f);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin({ startPos.x, startPos.y, startPos.z });

    // Kinematic body — we drive the transform ourselves
    motion_ = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo ci(
        0.f,    // mass=0 → static, but we add CF_KINEMATIC_OBJECT
        motion_,
        shape_,
        btVector3(0, 0, 0));
    body_ = new btRigidBody(ci);
    body_->setCollisionFlags(body_->getCollisionFlags() |
                              btCollisionObject::CF_KINEMATIC_OBJECT);
    body_->setActivationState(DISABLE_DEACTIVATION);

    world_->addRigidBody(body_);

    updateVectors();
}

Camera::~Camera()
{
    world_->removeRigidBody(body_);
    delete body_;
    delete motion_;
    delete shape_;
}

void Camera::updateVectors()
{
    float yawRad   = glm::radians(yaw_);
    float pitchRad = glm::radians(pitch_);

    front_.x = std::cos(yawRad) * std::cos(pitchRad);
    front_.y = std::sin(pitchRad);
    front_.z = std::sin(yawRad) * std::cos(pitchRad);
    front_   = glm::normalize(front_);

    right_ = glm::normalize(glm::cross(front_, glm::vec3(0.f, 1.f, 0.f)));
}

void Camera::processKeyboard(GLFWwindow* window)
{
    velocity_ = {0.f, 0.f, 0.f};

    if (!mouseCaptured) return;

    glm::vec3 flatFront = glm::normalize(glm::vec3(front_.x, 0.f, front_.z));

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) velocity_ += flatFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) velocity_ -= flatFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) velocity_ -= right_;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) velocity_ += right_;

    if (glfwGetKey(window, GLFW_KEY_SPACE)        == GLFW_PRESS) velocity_.y += 1.f;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) velocity_.y -= 1.f;

    if (glm::length(velocity_) > 0.001f)
        velocity_ = glm::normalize(velocity_);
}

void Camera::processMouse(double xoffset, double yoffset)
{
    if (!mouseCaptured) return;

    yaw_   += (float)xoffset * mouseSens_;
    pitch_ -= (float)yoffset * mouseSens_;
    pitch_  = std::clamp(pitch_, -89.f, 89.f);

    updateVectors();
}

void Camera::update(float dt)
{
    // Get current world position from body
    btTransform t;
    body_->getMotionState()->getWorldTransform(t);
    btVector3 pos = t.getOrigin();

    // Integrate velocity
    pos += btVector3(velocity_.x, velocity_.y, velocity_.z) * moveSpeed_ * dt;

    // Push new transform back to the kinematic body
    t.setOrigin(pos);
    body_->getMotionState()->setWorldTransform(t);
    body_->setWorldTransform(t);
}

glm::mat4 Camera::getViewMatrix() const
{
    glm::vec3 pos = getPosition();
    return glm::lookAt(pos, pos + front_, glm::vec3(0.f, 1.f, 0.f));
}

glm::vec3 Camera::getPosition() const
{
    btTransform t;
    body_->getMotionState()->getWorldTransform(t);
    btVector3 p = t.getOrigin();
    return { p.x(), p.y(), p.z() };
}

glm::vec3 Camera::getFront() const { return front_; }
