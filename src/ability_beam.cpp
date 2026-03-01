#include "ability_beam.h"
#include "shader.h"
#include "model.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <algorithm>
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
    // Sync beam lights to current physics positions
    if (!firedBeams_.empty()) {
        std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
        for (size_t i = 0; i < firedBeams_.size() && i < beamLightIdx_.size(); ++i) {
            size_t li = beamLightIdx_[i];
            if (li < ctx.lights.size())
                ctx.lights[li].position = firedBeams_[i].getPosition();
        }
    }

    // Detect Q-release edge → clean up session
    if (prevQHeld_ && !qHeld)
        cleanupSession(ctx);

    prevQHeld_ = qHeld_;
    qHeld_     = qHeld;
}

void BeamAbility::cleanupSession(const AbilityContext& ctx)
{
    // Remove lights in reverse index order so earlier indices stay valid
    std::sort(beamLightIdx_.begin(), beamLightIdx_.end(), std::greater<size_t>());
    for (size_t idx : beamLightIdx_) {
        if (idx < ctx.lights.size())
            ctx.lights.erase(ctx.lights.begin() + (ptrdiff_t)idx);
    }
    beamLightIdx_.clear();

    // Remove physics bodies (CollisionBox dtors call world->removeRigidBody)
    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    firedBeams_.clear();
}

void BeamAbility::onFire(const AbilityContext& ctx)
{
    if (!qHeld_ || !cubeModel_) return;

    glm::quat rot      = rotateZTo(ctx.camFront);
    glm::vec3 spawnPos = ctx.camPos + ctx.camFront * 2.5f;

    {
        std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
        firedBeams_.emplace_back(
            ctx.world, cubeModel_,
            glm::vec3{0.1f, 0.1f, 1.5f},
            spawnPos,
            glm::vec3{0.2f, 0.2f, 3.0f},
            glm::vec3{1.f, 1.f, 0.5f},
            10.f,
            rot,
            false);

        btRigidBody* b = firedBeams_.back().getBody();
        b->setGravity({0.f, 0.f, 0.f});
        b->setDamping(0.f, 0.f);
        b->setActivationState(DISABLE_DEACTIVATION);
        b->setLinearVelocity({
            ctx.camFront.x * fireVelocity,
            ctx.camFront.y * fireVelocity,
            ctx.camFront.z * fireVelocity
        });
    }

    ctx.lights.push_back({{spawnPos.x, spawnPos.y, spawnPos.z},
                           8.f, {1.f, 0.9f, 0.6f}, 15.f});
    beamLightIdx_.push_back(ctx.lights.size() - 1);
}

void BeamAbility::onDeselect()
{
    qHeld_ = false;
}

void BeamAbility::drawHUD(ImDrawList* dl, float cx, float cy)
{
    if (!qHeld_) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "BEAM [%d]", (int)firedBeams_.size());
    dl->AddText({cx - 30.f, cy + 10.f}, IM_COL32(255, 255, 150, 200), buf);
}
