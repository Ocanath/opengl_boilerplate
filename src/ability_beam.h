#pragma once
#include "ability.h"
#include "collision_box.h"
#include <vector>

class BeamAbility : public AbilityBase {
public:
    explicit BeamAbility(Model* cubeModel);

    const char* name() const override { return "BEAM"; }

    void update(float dt, const AbilityContext& ctx, bool qHeld) override;
    void onFire(const AbilityContext& ctx) override;
    void onDeselect() override;

    void drawPreview(Shader& unlitShader,
                     const glm::mat4& view, const glm::mat4& proj) override;

    void drawHUD(ImDrawList* dl, float cx, float cy) override;

    const std::vector<CollisionBox>* getBoxes() const override { return &firedBeams_; }

private:
    bool      previewing_   = false;
    glm::vec3 previewPos_   = {};
    glm::vec3 previewFront_ = {0.f, 0.f, 1.f};

    std::vector<CollisionBox> firedBeams_;
    std::vector<size_t>       beamLightIdx_;  // index into scene lights_ for each beam

    Model* cubeModel_;  // non-owning

    static glm::quat rotateZTo(const glm::vec3& target);
};
