#pragma once
#include "ability.h"
#include <vector>

class btRigidBody;

class GravitySwitchAbility : public AbilityBase {
public:
    const char* name() const override { return "GRAVITY"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;
    void onFire(const AbilityContext& ctx) override;
    void onDeselect() override;

    void drawHUD(ImDrawList* dl, float cx, float cy) override;

private:
    std::vector<btRigidBody*> selected_;
    bool                      qHeld_ = false;
};
