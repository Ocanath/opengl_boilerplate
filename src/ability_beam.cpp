#include "ability_beam.h"
#include "shader.h"
#include "model.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <mutex>
#include <cmath>

BeamAbility::BeamAbility(Model* cubeModel)
    : cubeModel_(cubeModel)
{}

// Compute a quaternion that rotates the local +Z axis to point toward `target`.
glm::quat BeamAbility::rotateZTo(const glm::vec3& target)
{
    glm::vec3 from = {0.f, 0.f, 1.f};
    glm::vec3 to   = glm::normalize(target);
    float cosA = glm::dot(from, to);

    if (cosA > 0.9999f)
        return glm::quat(1.f, 0.f, 0.f, 0.f);

    if (cosA < -0.9999f)
        return glm::angleAxis(glm::pi<float>(), glm::vec3(1.f, 0.f, 0.f));

    glm::vec3 axis  = glm::normalize(glm::cross(from, to));
    float     angle = std::acos(glm::clamp(cosA, -1.f, 1.f));
    return glm::angleAxis(angle, axis);
}

void BeamAbility::update(float /*dt*/, const AbilityContext& ctx, bool qHeld)
{
    previewing_   = qHeld;
    previewPos_   = ctx.camPos;
    previewFront_ = ctx.camFront;

    // Sync beam lights to current physics positions
    if (!firedBeams_.empty()) {
        std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
        for (size_t i = 0; i < firedBeams_.size() && i < beamLightIdx_.size(); ++i) {
            size_t li = beamLightIdx_[i];
            if (li < ctx.lights.size())
                ctx.lights[li].position = firedBeams_[i].getPosition();
        }
    }
}

void BeamAbility::onFire(const AbilityContext& ctx)
{
    if (!previewing_ || !cubeModel_) return;

    glm::quat rot      = rotateZTo(previewFront_);
    glm::vec3 spawnPos = previewPos_ + previewFront_ * 2.5f;

    {
        std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
        firedBeams_.emplace_back(
            ctx.world, cubeModel_,
            glm::vec3{0.1f, 0.1f, 1.5f},
            spawnPos,
            glm::vec3{0.2f, 0.2f, 3.0f},
            glm::vec3{1.f, 1.f, 0.5f},
            1.f,
            rot,
            false);

        btRigidBody* b = firedBeams_.back().getBody();
        b->setGravity({0.f, 0.f, 0.f});
        b->setDamping(0.f, 0.f);
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setLinearVelocity({
            previewFront_.x * 20.f,
            previewFront_.y * 20.f,
            previewFront_.z * 20.f
        });
    }

    ctx.lights.push_back({{spawnPos.x, spawnPos.y, spawnPos.z},
                           8.f, {1.f, 0.9f, 0.6f}, 15.f});
    beamLightIdx_.push_back(ctx.lights.size() - 1);
}

void BeamAbility::onDeselect()
{
    previewing_ = false;
}

void BeamAbility::drawPreview(Shader& shader,
                               const glm::mat4& /*view*/, const glm::mat4& /*proj*/)
{
    if (!previewing_ || !cubeModel_) return;

    glm::vec3 center = previewPos_ + previewFront_ * 2.5f;
    glm::quat rot    = rotateZTo(previewFront_);

    glm::mat4 model =
        glm::translate(glm::mat4(1.f), center)
        * glm::mat4_cast(rot)
        * glm::scale(glm::mat4(1.f), {0.2f, 0.2f, 3.0f});

    shader.setMat4("model",       model);
    shader.setVec3("objectColor", {1.f, 1.f, 0.6f});
    cubeModel_->draw(shader);
}

void BeamAbility::drawHUD(ImDrawList* dl, float cx, float cy)
{
    if (!previewing_) return;
    dl->AddText({cx - 20.f, cy + 10.f}, IM_COL32(255, 255, 150, 200), "BEAM");
}
