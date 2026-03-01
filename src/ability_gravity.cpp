#include "ability_gravity.h"
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include <imgui.h>
#include <mutex>
#include <cmath>
#include <cstdio>
#include <algorithm>

// ── Private helpers (called under physics mutex) ──────────────────────────────

void GravitySwitchAbility::disableGravity(btRigidBody* body)
{
    // Only act if gravity is currently non-zero
    if (body->getGravity().length() <= 0.1f) return;

    body->setGravity({0.f, 0.f, 0.f});
    body->activate(true);

    // Small upward force for kBoostTime seconds
    TimedEffect boost{};
    boost.body     = body;
    boost.timeLeft = kBoostTime;
    boost.kind     = TimedEffect::Kind::ContinuousForce;
    boost.forceZ   = kBoostForceZ;
    effects_.push_back(boost);

    // High drag for kDragTime seconds, then restore original damping
    TimedEffect drag{};
    drag.body         = body;
    drag.timeLeft     = kDragTime;
    drag.kind         = TimedEffect::Kind::RestoreDamping;
    drag.savedLinDamp = body->getLinearDamping();
    drag.savedAngDamp = body->getAngularDamping();
    body->setDamping(kHighLinDamp, kHighAngDamp);
    effects_.push_back(drag);
}

void GravitySwitchAbility::enableGravity(btRigidBody* body)
{
    // Only act if gravity is currently zero
    if (body->getGravity().length() > 0.1f) return;

    // Apply N G's for kGravTime seconds, then drop back to 1G
    body->setGravity({0.f, 0.f, -9.8f * kExtraGravN});
    body->activate(true);

    TimedEffect grav{};
    grav.body     = body;
    grav.timeLeft = kGravTime;
    grav.kind     = TimedEffect::Kind::RestoreGravity;
    effects_.push_back(grav);
}

// ── AbilityBase interface ─────────────────────────────────────────────────────

void GravitySwitchAbility::update(float dt, const AbilityContext& ctx, bool qHeld)
{
    qHeld_ = qHeld;
    selected_.clear();

    bool needMutex = !effects_.empty() || qHeld;
    if (!needMutex) return;

    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);

    // ── Tick and apply timed effects ─────────────────────────────────────────
    for (auto& e : effects_) {
        e.timeLeft -= dt;
        if (e.kind == TimedEffect::Kind::ContinuousForce && e.timeLeft > 0.f)
            e.body->applyCentralForce({0.f, 0.f, e.forceZ});
    }

    // Expire: restore physics properties and erase
    effects_.erase(
        std::remove_if(effects_.begin(), effects_.end(), [](TimedEffect& e) {
            if (e.timeLeft > 0.f) return false;
            switch (e.kind) {
                case TimedEffect::Kind::RestoreDamping:
                    e.body->setDamping(e.savedLinDamp, e.savedAngDamp);
                    break;
                case TimedEffect::Kind::RestoreGravity:
                    e.body->setGravity({0.f, 0.f, -9.8f});
                    break;
                default: break;
            }
            return true;
        }),
        effects_.end());

    if (!qHeld) return;

    // ── Project dynamic bodies → screen, collect those within 50px of centre ─
    auto& arr = ctx.world->getCollisionObjectArray();
    for (int i = 0; i < arr.size(); ++i) {
        btCollisionObject* obj = arr[i];
        if (obj->isStaticOrKinematicObject()) continue;

        btRigidBody* body = btRigidBody::upcast(obj);
        if (!body) continue;

        btVector3 btPos = obj->getWorldTransform().getOrigin();
        glm::vec4 clip  = ctx.proj * ctx.view
                          * glm::vec4(btPos.x(), btPos.y(), btPos.z(), 1.f);

        if (clip.w <= 0.f) continue;

        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        if (ndc.z > 1.f) continue;

        float screenX = (ndc.x * 0.5f + 0.5f) * (float)ctx.viewW;
        float screenY = (1.f - (ndc.y * 0.5f + 0.5f)) * (float)ctx.viewH;

        float dx = screenX - (float)ctx.viewW * 0.5f;
        float dy = screenY - (float)ctx.viewH * 0.5f;

        if (std::sqrt(dx * dx + dy * dy) <= 50.f)
            selected_.push_back(body);
    }
}

void GravitySwitchAbility::onFire(const AbilityContext& ctx)
{
    if (selected_.empty()) return;
    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    for (btRigidBody* body : selected_)
        disableGravity(body);
    selected_.clear();
}

void GravitySwitchAbility::onFireSecondary(const AbilityContext& ctx)
{
    if (selected_.empty()) return;
    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    for (btRigidBody* body : selected_)
        enableGravity(body);
    selected_.clear();
}

void GravitySwitchAbility::onDeselect()
{
    selected_.clear();
    qHeld_ = false;
}

void GravitySwitchAbility::drawHUD(ImDrawList* dl, float cx, float cy)
{
    if (!qHeld_) return;

    ImU32 colour = selected_.empty() ? IM_COL32(180, 180, 180, 150)
                                     : IM_COL32(50, 220, 50, 200);

    dl->AddCircle({cx, cy}, 50.f, colour, 32, 2.f);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "x%d", (int)selected_.size());
    dl->AddText({cx - 10.f, cy + 55.f}, colour, buf);

    // Remind the player which button does what
    dl->AddText({cx - 55.f, cy + 68.f},
                IM_COL32(200, 200, 200, 180), "LMB=off  RMB=on");
}
