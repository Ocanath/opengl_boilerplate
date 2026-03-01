#pragma once
#include "ability.h"
#include <vector>

class btRigidBody;

class PushAbility : public AbilityBase {
public:
    const char* name() const override { return "PUSH"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;
    void onFire(const AbilityContext& ctx) override;
    void onFireSecondary(const AbilityContext& ctx) override;
    void onDeselect() override;
    void drawHUD(ImDrawList* dl, float cx, float cy) override;

    float pushStrength = 50.f;  // impulse magnitude (N·s)

private:
    std::vector<btRigidBody*> selected_;
    bool qHeld_ = false;

    static constexpr float kSelectionRadius = 100.f;
};
