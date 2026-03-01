#pragma once
#include <glm/glm.hpp>

// Forward-declare Bullet types to avoid pulling in all headers
class btRigidBody;
class btCapsuleShape;
class btMotionState;
class btDiscreteDynamicsWorld;
struct GLFWwindow;

class Camera
{
public:
    Camera(btDiscreteDynamicsWorld* world,
           glm::vec3 startPos = {0.f, 2.f, 5.f});
    ~Camera();

    // Call each frame before rendering
    void update(float dt);

    // Pass GLFW events to the camera
    void processKeyboard(GLFWwindow* window);
    void processMouse(double xoffset, double yoffset);

    glm::mat4 getViewMatrix()  const;
    glm::vec3 getPosition()    const;
    glm::vec3 getFront()       const;

    bool mouseCaptured = true;

private:
    btDiscreteDynamicsWorld* world_;
    btCapsuleShape*  shape_  = nullptr;
    btRigidBody*     body_   = nullptr;
    btMotionState*   motion_ = nullptr;

    glm::vec3 velocity_   = {0.f, 0.f, 0.f};
    float     yaw_        = -90.f;
    float     pitch_      = 0.f;
    float     moveSpeed_  = 6.f;
    float     mouseSens_  = 0.12f;

    glm::vec3 front_      = {0.f, 0.f, -1.f};
    glm::vec3 right_      = {1.f, 0.f,  0.f};
    void updateVectors();
};
