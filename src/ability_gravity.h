#pragma once
#include "ability.h"
#include <vector>

class btRigidBody;

class GravitySwitchAbility : public AbilityBase {
public:
    const char* name() const override { return "GRAVITY"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;

    // LMB: disable gravity on selected bodies
    void onFire(const AbilityContext& ctx) override;

    // RMB: enable gravity on selected bodies
    void onFireSecondary(const AbilityContext& ctx) override;

    void onDeselect() override;

    void drawHUD(ImDrawList* dl, float cx, float cy) override;

private:
    // ── Timed per-body effect ─────────────────────────────────────────────────
    struct TimedEffect {
        btRigidBody* body;
        float        timeLeft;  // seconds; tick down in update()

        enum class Kind {
            ContinuousForce,  // apply forceZ every frame while timeLeft > 0
            RestoreDamping,   // on expiry: restore saved damping
            RestoreGravity,   // on expiry: restore gravity to 1G
        } kind;

        // ContinuousForce payload
        float forceZ = 0.f;

        // RestoreDamping payload
        float savedLinDamp = 0.f;
        float savedAngDamp = 0.f;
    };

    std::vector<btRigidBody*> selected_;
    std::vector<TimedEffect>  effects_;
    bool                      qHeld_ = false;

    // Tuning knobs
    static constexpr float kExtraGravN  = 50.f;   // G multiplier on enable (3G)
    static constexpr float kGravTime    = 0.1f;  // seconds at N G's before 1G
    static constexpr float kBoostForceZ = 20.f;  // N upward on disable
    static constexpr float kBoostTime   = 0.5f;  // seconds of upward force
    static constexpr float kDragTime    = 2.0f;  // seconds of high drag
    static constexpr float kHighLinDamp = 0.95f;
    static constexpr float kHighAngDamp = 0.5f;

    // Call under physics mutex
    void disableGravity(btRigidBody* body);
    void enableGravity (btRigidBody* body);
};
