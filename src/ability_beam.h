#pragma once
#include "ability.h"
#include "collision_box.h"
#include <vector>
#include <unordered_set>

class BeamAbility : public AbilityBase {
public:
    explicit BeamAbility(Model* cubeModel);

    const char* name() const override { return "BEAM"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;
    void onFire(const AbilityContext& ctx) override;
    void onDeselect() override;
    void onKeyPress(int key, const AbilityContext& ctx) override;

    void drawHUD(ImDrawList* dl, float cx, float cy) override;

    const std::vector<CollisionBox>* getBoxes() const override { return &firedBeams_; }

    float fireVelocity = 100.f;  // exposed for UI slider

private:
    bool qHeld_     = false;
    bool prevQHeld_ = false;

    std::vector<CollisionBox> firedBeams_;
    std::vector<size_t>       beamLightIdx_;  // index into scene lights_ for each beam
    std::unordered_set<btRigidBody*> hitBodies_;  // all dynamic bodies ever touched by a beam

    Model* cubeModel_;  // non-owning

    void cleanupSession(const AbilityContext& ctx);
    static glm::quat rotateZTo(const glm::vec3& target);
};
