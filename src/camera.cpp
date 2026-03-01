#include "camera.h"
#include <GLFW/glfw3.h>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>
#include <btBulletDynamicsCommon.h>
#include <algorithm>
#include <cmath>

Camera::Camera(btDiscreteDynamicsWorld* world, glm::vec3 startPos)
    : world_(world)
{
    // Box shape: half extents (1, 2.5, 1) — 2×5×2 bounding box
    shape_ = new btBoxShape({ 1.f, 1.f, 2.5f });

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
    front_.y = std::sin(yawRad) * std::cos(pitchRad);
    front_.z = std::sin(pitchRad);
    front_   = glm::normalize(front_);

    right_ = glm::normalize(glm::cross(front_, glm::vec3(0.f, 0.f, 1.f)));
}

void Camera::processKeyboard(GLFWwindow* window)
{
    velocity_        = {0.f, 0.f, 0.f};
    vertInputActive_ = false;

    if (!mouseCaptured) return;

    glm::vec3 flatFront = glm::normalize(glm::vec3(front_.x, front_.y, 0.f));

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) velocity_ += flatFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) velocity_ -= flatFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) velocity_ -= right_;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) velocity_ += right_;

    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        velocity_.z      += 1.f;
        vertInputActive_  = true;
    }
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        velocity_.z      -= 1.f;
        vertInputActive_  = true;
    }

    // Normalize horizontal XY component independently
    glm::vec3 horiz = { velocity_.x, velocity_.y, 0.f };
    if (glm::length(horiz) > 0.001f)
        horiz = glm::normalize(horiz);
    velocity_.x = horiz.x;
    velocity_.y = horiz.y;
}

void Camera::applyVelocity()
{
    btVector3 cur = body_->getLinearVelocity();
    float vx = velocity_.x * moveSpeed_;
    float vy = velocity_.y * moveSpeed_;
    // Preserve physics-driven Z (gravity) when active; snap to 0 when gravity is off and no key held
    float vz = vertInputActive_ ? velocity_.z * moveSpeed_ : (gravityEnabled ? cur.z() : 0.f);
    body_->setLinearVelocity({ vx, vy, vz });
    body_->activate(true);
}

void Camera::processMouse(double xoffset, double yoffset)
{
    if (!mouseCaptured) return;

    yaw_   -= (float)xoffset * mouseSens_;
    pitch_ -= (float)yoffset * mouseSens_;
    pitch_  = std::clamp(pitch_, -89.f, 89.f);

    updateVectors();
}

glm::mat4 Camera::getViewMatrix() const
{
    glm::vec3 pos = getPosition();
    return glm::lookAt(pos, pos + front_, glm::vec3(0.f, 0.f, 1.f));
}

glm::vec3 Camera::getPosition() const
{
    btTransform t;
    body_->getMotionState()->getWorldTransform(t);
    btVector3 p = t.getOrigin();
    return { p.x(), p.y(), p.z() };
}

glm::vec3 Camera::getFront() const { return front_; }

void Camera::saveToFile(const std::string& path) const
{
    glm::vec3 pos = getPosition();
    std::ofstream f(path);
    if (f) f << pos.x << ' ' << pos.y << ' ' << pos.z
             << ' ' << yaw_ << ' ' << pitch_ << '\n';
}

void Camera::loadFromFile(const std::string& path)
{
    std::ifstream f(path);
    if (!f) return;
    float x, y, z, yaw, pitch;
    if (!(f >> x >> y >> z >> yaw >> pitch)) return;

    btTransform t;
    t.setIdentity();
    t.setOrigin({ x, y, z });
    body_->setWorldTransform(t);
    motion_->setWorldTransform(t);
    body_->clearForces();
    body_->setLinearVelocity({ 0.f, 0.f, 0.f });
    body_->setAngularVelocity({ 0.f, 0.f, 0.f });
    body_->activate(true);

    yaw_   = yaw;
    pitch_ = pitch;
    updateVectors();
}

void Camera::setGravity(bool enabled)
{
    gravityEnabled = enabled;
    body_->setGravity(enabled ? world_->getGravity() : btVector3(0.f, 0.f, 0.f));
    if (!enabled) {
        btVector3 v = body_->getLinearVelocity();
        body_->setLinearVelocity({ v.x(), v.y(), 0.f });
    }
}
