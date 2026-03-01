#include "ability_gravity.h"
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include <imgui.h>
#include <mutex>
#include <cmath>
#include <cstdio>

void GravitySwitchAbility::update(float /*dt*/, const AbilityContext& ctx, bool qHeld)
{
    qHeld_ = qHeld;
    selected_.clear();

    if (!qHeld) return;

    // Project every dynamic body onto the screen, collect those within 50px of centre
    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
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
        if (ndc.z > 1.f) continue;  // behind far plane

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
    for (btRigidBody* body : selected_) {
        btVector3 g       = body->getGravity();
        bool      hasGrav = g.length() > 0.1f;
        body->setGravity(hasGrav ? btVector3(0.f, 0.f, 0.f)
                                 : btVector3(0.f, 0.f, -9.8f));
        body->activate(true);
    }
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
}
