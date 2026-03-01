#include "camera.h"
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <btBulletDynamicsCommon.h>
#include <algorithm>
#include <cmath>

Camera::Camera(btDiscreteDynamicsWorld* world, glm::vec3 startPos)
    : world_(world)
{
    // Box shape: half extents (1, 2.5, 1) — 2×5×2 bounding box
    shape_ = new btBoxShape({ 1.f, 2.5f, 1.f });

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin({ startPos.x, startPos.y, startPos.z });

    // Dynamic body — gravity applies
    btVector3 inertia(0, 0, 0);
    shape_->calculateLocalInertia(80.f, inertia);

    motion_ = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo ci(80.f, motion_, shape_, inertia);
    body_ = new btRigidBody(ci);

    // Prevent tipping — lock all rotational axes
    body_->setAngularFactor({ 0.f, 0.f, 0.f });

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
    velocity_        = {0.f, 0.f, 0.f};
    vertInputActive_ = false;

    if (!mouseCaptured) return;

    glm::vec3 flatFront = glm::normalize(glm::vec3(front_.x, 0.f, front_.z));

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) velocity_ += flatFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) velocity_ -= flatFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) velocity_ -= right_;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) velocity_ += right_;

    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        velocity_.y      += 1.f;
        vertInputActive_  = true;
    }
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        velocity_.y      -= 1.f;
        vertInputActive_  = true;
    }

    // Normalize horizontal component independently
    glm::vec3 horiz = { velocity_.x, 0.f, velocity_.z };
    if (glm::length(horiz) > 0.001f)
        horiz = glm::normalize(horiz);
    velocity_.x = horiz.x;
    velocity_.z = horiz.z;
}

void Camera::applyVelocity()
{
    btVector3 cur = body_->getLinearVelocity();
    float vx = velocity_.x * moveSpeed_;
    float vz = velocity_.z * moveSpeed_;
    // Preserve physics-driven Y velocity unless player explicitly controls it
    float vy = vertInputActive_ ? velocity_.y * moveSpeed_ : cur.y();
    body_->setLinearVelocity({ vx, vy, vz });
    body_->activate(true);
}

void Camera::processMouse(double xoffset, double yoffset)
{
    if (!mouseCaptured) return;

    yaw_   += (float)xoffset * mouseSens_;
    pitch_ -= (float)yoffset * mouseSens_;
    pitch_  = std::clamp(pitch_, -89.f, 89.f);

    updateVectors();
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
