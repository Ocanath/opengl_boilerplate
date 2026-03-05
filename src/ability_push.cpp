#include "ability_push.h"
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include <imgui.h>
#include <mutex>
#include <cmath>
#include <cstdio>

void PushAbility::update(float dt, const AbilityContext& ctx, bool qHeld)
{
    qHeld_ = qHeld;
    selected_.clear();

    if (!qHeld) return;

    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);

    auto& arr = ctx.world->getCollisionObjectArray();
    for (int i = 0; i < arr.size(); ++i) 
	{
        btCollisionObject* obj = arr[i];
        if (obj->isStaticOrKinematicObject()) continue;

        btRigidBody* body = btRigidBody::upcast(obj);
        if (!body) continue;
        if (body == ctx.cameraBody) continue;

        if (selectAll) {
            selected_.push_back(body);
            continue;
        }

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

        if (std::sqrt(dx * dx + dy * dy) <= selectionRadius)
            selected_.push_back(body);
    }
}

void PushAbility::onFire(const AbilityContext& ctx)
{
    if (!qHeld_ || selected_.empty()) return;

    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    btVector3 impulse(ctx.camFront.x * pushStrength,
                      ctx.camFront.y * pushStrength,
                      ctx.camFront.z * pushStrength);
    for (btRigidBody* body : selected_) {
        body->activate(true);
        body->applyCentralImpulse(impulse);
    }
    selected_.clear();
}

void PushAbility::onFireSecondary(const AbilityContext& ctx)
{
    if (!qHeld_ || selected_.empty()) return;

    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    btVector3 impulse(-ctx.camFront.x * pushStrength,
                      -ctx.camFront.y * pushStrength,
                      -ctx.camFront.z * pushStrength);
    for (btRigidBody* body : selected_) {
        body->activate(true);
        body->applyCentralImpulse(impulse);
    }
    selected_.clear();
}

void PushAbility::onDeselect()
{
    selected_.clear();
    qHeld_ = false;
}

void PushAbility::drawHUD(ImDrawList* dl, float cx, float cy)
{
	return;
    if (!qHeld_) return;

    ImU32 colour = selected_.empty() ? IM_COL32(180, 180, 180, 150)
                                     : IM_COL32(50, 220, 50, 200);

    dl->AddCircle({cx, cy}, selectionRadius, colour, 32, 2.f);

    char buf[32];
    std::snprintf(buf, sizeof(buf), "x%d", (int)selected_.size());
    dl->AddText({cx - 10.f, cy + selectionRadius + 5.f}, colour, buf);
}
