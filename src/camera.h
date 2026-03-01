#pragma once
#include <glm/glm.hpp>

// Forward-declare Bullet types to avoid pulling in all headers
class btRigidBody;
class btBoxShape;
class btMotionState;
class btDiscreteDynamicsWorld;
struct GLFWwindow;

class Camera
{
public:
    Camera(btDiscreteDynamicsWorld* world,
           glm::vec3 startPos = {0.f, 5.f, 0.f});
    ~Camera();

    // CPU-only: compute desired velocity from key state (no mutex needed)
    void processKeyboard(GLFWwindow* window);

    // Apply computed velocity to physics body — call under physics mutex
    void applyVelocity();

    void processMouse(double xoffset, double yoffset);

    glm::mat4 getViewMatrix() const;
    glm::vec3 getPosition()   const;
    glm::vec3 getFront()      const;

    bool mouseCaptured = true;

private:
    btDiscreteDynamicsWorld* world_;
    btBoxShape*    shape_  = nullptr;
    btRigidBody*   body_   = nullptr;
    btMotionState* motion_ = nullptr;

    glm::vec3 velocity_        = {0.f, 0.f, 0.f};
    bool      vertInputActive_ = false;
    float     yaw_             = -90.f;
    float     pitch_           = 0.f;
    float     moveSpeed_       = 6.f;
    float     mouseSens_       = 0.12f;

    glm::vec3 front_ = {0.f, 0.f, -1.f};
    glm::vec3 right_ = {1.f, 0.f,  0.f};
    void updateVectors();
};
