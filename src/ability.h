#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <mutex>
#include <functional>
#include <unordered_map>
#include "light.h"

class Shader;
class Model;
class CollisionBox;
class btDiscreteDynamicsWorld;
class btRigidBody;
struct ImDrawList;

// Per-frame context passed to ability update/fire
struct AbilityContext {
    btDiscreteDynamicsWorld* world;
    std::mutex*              physicsMutex;
    Model*                   cubeModel;      // shared unit cube for spawning boxes

    glm::vec3 camPos;
    glm::vec3 camFront;
    glm::mat4 view;
    glm::mat4 proj;
    int       viewW, viewH;

    bool lmbHeld = false;        // true when LMB held and mouse is captured
    bool fHeld   = false;        // true when F held and mouse is captured
    bool rHeld   = false;        // true when R held and mouse is captured

    std::vector<Light>& lights;  // scene's main light list (abilities may append)
    std::function<void(btRigidBody*)> removeBody;  // remove a dynamic body from the scene

    struct InitialBodyState { glm::vec3 pos; glm::quat rot; };
    const std::unordered_map<btRigidBody*, InitialBodyState>* initialBodyStates = nullptr;
    btRigidBody* cameraBody = nullptr;  // excluded from selection
};

class AbilityBase {
public:
    virtual ~AbilityBase() = default;
    virtual const char* name() const = 0;

    // Selection tuning — public for UI overlay
    float selectionRadius = 100.f;
    bool  selectAll       = false;

    // Called every frame. qHeld = Q is currently pressed.
    virtual void update(float dt, const AbilityContext& ctx, bool qHeld) = 0;

    // Called on LMB press (while mouse is captured).
    virtual void onFire(const AbilityContext& ctx) = 0;

    // Called on RMB press (while mouse is captured). Default: no-op.
    virtual void onFireSecondary(const AbilityContext& ctx) {}

    // Called when the player switches away from this ability.
    virtual void onDeselect() = 0;

    // Called on key press while mouse is captured.
    virtual void onKeyPress(int /*key*/, const AbilityContext& /*ctx*/) {}

    // Called on mouse scroll (delta in scroll ticks, positive = up).
    virtual void onScroll(float delta) {}

    // Render any 3D preview geometry (called in Pass 3 unlit pass; shader already bound).
    virtual void drawPreview(Shader& unlitShader,
                             const glm::mat4& view, const glm::mat4& proj) {}

    // Draw HUD overlays via ImGui DrawList (cx/cy = screen centre in pixels).
    virtual void drawHUD(ImDrawList* dl, float cx, float cy) {}

    // Draw ImGui overlay window contents (called inside an ImGui::Begin/End block).
    virtual void drawOverlay() {}

    // Scene calls this to collect ability-owned CollisionBoxes for the unlit render pass.
    virtual const std::vector<CollisionBox>* getBoxes() const { return nullptr; }
};
