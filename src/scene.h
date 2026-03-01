#pragma once
#include <vector>
#include <string>
#include <memory>
#include "model.h"
#include "camera.h"
#include "light.h"
#include "shader.h"

// Forward-declare Bullet types
class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btRigidBody;

class Scene
{
public:
    Scene();
    ~Scene();

    void addModel(const std::string& path);
    void addLight(const Light& light);
    void addGroundPlane(float y = 0.f);

    void update(float dt, struct GLFWwindow* window);
    void draw(Shader& shader, int viewportWidth, int viewportHeight);

    Camera& getCamera() { return *camera_; }
    std::vector<Light>& getLights() { return lights_; }

private:
    // Bullet objects (owned)
    btBroadphaseInterface*               broadphase_  = nullptr;
    btDefaultCollisionConfiguration*     collConfig_  = nullptr;
    btCollisionDispatcher*               dispatcher_  = nullptr;
    btSequentialImpulseConstraintSolver* solver_      = nullptr;
    btDiscreteDynamicsWorld*             dynamicsWorld_ = nullptr;

    btRigidBody* groundBody_ = nullptr;

    std::vector<Model>  models_;
    std::vector<Light>  lights_;
    std::unique_ptr<Camera> camera_;
};
