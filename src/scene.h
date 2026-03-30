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
#include "ability.h"
#include "lidar.h"

struct ImDrawList;

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
	void addPile(glm::vec3 loc);
    
    void update(float dt, struct GLFWwindow* window);
    void draw(int viewportWidth, int viewportHeight);

    Camera& getCamera() { return *camera_; }
    std::vector<Light>& getLights() { return lights_; }
    LidarSystem& getLidar() { return *lidar_; }

    // Thread-safe camera position read
    glm::vec3 getCameraPosition();

    // Thread-safe camera gravity toggle
    void setCameraGravity(bool enabled);

    void saveCameraToFile(const std::string& path);
    void loadCameraFromFile(const std::string& path);

	std::vector<CollisionBox> floatingPillars_; // 5 static pillars
	
    // Ability system
    void selectAbility(int index);   // 0-based
    void firePrimary();              // on LMB press
    void fireSecondary();            // on RMB press
    void drawActiveAbilityHUD(ImDrawList* dl, float cx, float cy);
    void onScroll(float delta);
    void drawActiveAbilityOverlay();
    int         getActiveAbility() const { return activeAbility_; }
    int         getAbilityCount()  const { return (int)abilities_.size(); }
    const char* getAbilityName(int i) const;
    float&      beamFireVelocity();      // ref to BeamAbility::fireVelocity for UI
    std::unique_ptr<Model>  cubeModel_;   // shared unit cube for all box meshes
	btDiscreteDynamicsWorld*             dynamicsWorld_ = nullptr;

private:
    // Bullet objects (owned)
    btBroadphaseInterface*               broadphase_    = nullptr;
    btDefaultCollisionConfiguration*     collConfig_    = nullptr;
    btCollisionDispatcher*               dispatcher_    = nullptr;
    btSequentialImpulseConstraintSolver* solver_        = nullptr;
    

    // Physics thread
    std::mutex        physicsMutex_;
    std::thread       physicsThread_;
    std::atomic<bool> physicsRunning_{false};
    void physicsLoop();

    // Scene objects
    std::vector<Model>      models_;
    std::vector<Light>      lights_;
    std::unique_ptr<Camera> camera_;

    // Collision boxes
    std::vector<CollisionBox> lightBoxes_;      // kinematic, one per light
    std::vector<CollisionBox> chamberWalls_;    // 6 static slabs
    // GPU ring buffer for point cloud — never re-uploaded, only appended
    static constexpr int MAX_GPU_POINTS = 5'000'000; // ~60 MB, ~22 s of history at 600 Hz
    int gpuWriteHead_ = 0;   // next write position (in points)
    int gpuTotalPts_  = 0;   // total valid points (capped at MAX_GPU_POINTS)

    // LiDAR
    std::unique_ptr<LidarSystem> lidar_;
    void updateLidarPoints();
	
    // Ability system
    std::vector<std::unique_ptr<AbilityBase>> abilities_;
    int activeAbility_ = 0;

    // Per-frame camera snapshot (set in update, consumed by ability system)
    glm::vec3 camPos_    = {};
    glm::vec3 camFront_  = {1.f, 0.f, 0.f};
    glm::mat4 lastView_  = glm::mat4(1.f);
    glm::mat4 lastProj_  = glm::mat4(1.f);
    int       lastViewW_ = 0;
    int       lastViewH_ = 0;

    void buildChamber();
    void buildPillars();

    // ── Deferred rendering ────────────────────────────────────────────────
    // G-buffer
    unsigned int gFBO_      = 0;
    unsigned int gPos_      = 0;   // RGBA16F texture: world position
    unsigned int gNorm_     = 0;   // RGBA16F texture: world normal
    unsigned int gAlbedo_   = 0;   // RGBA8   texture: albedo
    unsigned int gDepthRBO_ = 0;   // D24S8 renderbuffer
    int          gWidth_    = 0;
    int          gHeight_   = 0;

    // Light SSBO
    unsigned int lightSSBO_ = 0;

    // Fullscreen quad
    unsigned int quadVAO_ = 0;
    unsigned int quadVBO_ = 0;

    // Owned shaders
    std::unique_ptr<Shader> gShader_;        // G-buffer pass
    std::unique_ptr<Shader> lightingShader_; // Lighting pass
    std::unique_ptr<Shader> unlitShader_;    // Forward unlit pass
    std::unique_ptr<Shader> pointCloudShader_;

    // Point cloud GL objects
    unsigned int pointCloudVAO_ = 0;
    unsigned int pointCloudVBO_ = 0;

    void initGBuffer(int w, int h);
    void destroyGBuffer();
    void initQuad();
    void uploadLights();
};
