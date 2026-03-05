#pragma once
#include "ability.h"
#include <vector>
#include <glm/glm.hpp>

class btRigidBody;

class MoveAbility : public AbilityBase {
public:
    const char* name() const override { return "MOVE"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;
    void onFire(const AbilityContext& ctx) override;
    void onKeyPress(int key, const AbilityContext& ctx) override;
    void onDeselect() override;
    void onScroll(float delta) override;
    void drawHUD(ImDrawList* dl, float cx, float cy) override;
    void drawOverlay() override;

    // PID tuning — public for overlay
    float Kp             = 20.f;
    float Ki             = 0.f;
    float Kd             = 5.f;
    float maxForce       = 1000.f;
    float grabDist       = 50.f;
    float explodeStrength = 300.f;
    float KpRot          = 10.f;
    float KdRot          =  2.f;
    float maxTorque      = 500.f;

private:
    static std::vector<glm::vec3> fibonacciSphere(int n);
    struct GrabbedBody {
        btRigidBody* body;
        glm::vec3    integral  = {};
        glm::vec3    prevError = {};
        glm::vec3    fibOffset = {};   // unit direction on Fibonacci sphere (set at grab time)
    };

    std::vector<btRigidBody*> selected_;   // Q-selection, refreshed each frame
    std::vector<GrabbedBody>  grabbed_;    // locked on LMB press, held while LMB held
    bool  qHeld_   = false;
    bool  lmbHeld_ = false;
    bool  rHeld_   = false;
    float fRadius_ = 0.f;       // current interpolated formation radius

    static constexpr float kMinDist = 2.f;
    static constexpr float kMaxDist = 200.f;
};
