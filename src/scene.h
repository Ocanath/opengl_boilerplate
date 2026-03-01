#pragma once
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include "model.h"
#include "camera.h"
#include "light.h"
#include "shader.h"
#include "collision_box.h"

// Forward-declare Bullet types
class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;

class Scene
{
public:
    Scene();
    ~Scene();

    void addModel(const std::string& path);
    void addLight(const Light& light);

    void update(float dt, struct GLFWwindow* window);
    void draw(Shader& shader, int viewportWidth, int viewportHeight);

    Camera& getCamera() { return *camera_; }
    std::vector<Light>& getLights() { return lights_; }

    // Thread-safe camera position read
    glm::vec3 getCameraPosition();

    // Thread-safe camera gravity toggle
    void setCameraGravity(bool enabled);

    void saveCameraToFile(const std::string& path);
    void loadCameraFromFile(const std::string& path);

private:
    // Bullet objects (owned)
    btBroadphaseInterface*               broadphase_    = nullptr;
    btDefaultCollisionConfiguration*     collConfig_    = nullptr;
    btCollisionDispatcher*               dispatcher_    = nullptr;
    btSequentialImpulseConstraintSolver* solver_        = nullptr;
    btDiscreteDynamicsWorld*             dynamicsWorld_ = nullptr;

    // Physics thread
    std::mutex        physicsMutex_;
    std::thread       physicsThread_;
    std::atomic<bool> physicsRunning_{false};
    void physicsLoop();

    // Scene objects
    std::unique_ptr<Model>  cubeModel_;   // shared unit cube for all box meshes
    std::vector<Model>      models_;
    std::vector<Light>      lights_;
    std::unique_ptr<Camera> camera_;

    // Collision boxes
    std::vector<CollisionBox> lightBoxes_;      // kinematic, one per light
    std::vector<CollisionBox> chamberWalls_;    // 6 static slabs
    std::vector<CollisionBox> floatingPillars_; // 5 static pillars

    void buildChamber();
    void buildPillars();
};
