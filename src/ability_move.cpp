#include "ability_move.h"
#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <mutex>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<glm::vec3> MoveAbility::fibonacciSphere(int n)
{
    std::vector<glm::vec3> pts;
    pts.reserve(n);
    const float golden = (float)M_PI * (3.f - std::sqrt(5.f));
    for (int i = 0; i < n; ++i) {
        float y = 1.f - (2.f * i + 1.f) / (float)n;
        float r = std::sqrt(std::max(0.f, 1.f - y * y));
        float t = golden * (float)i;
        pts.push_back({ r * std::cos(t), r * std::sin(t), y });
    }
    return pts;
}

void MoveAbility::onKeyPress(int key, const AbilityContext& ctx)
{
    if (key != GLFW_KEY_E) return;

    std::vector<btRigidBody*> targets;
    if (!grabbed_.empty())
        for (auto& gb : grabbed_) targets.push_back(gb.body);
    else if (!selected_.empty())
        targets = selected_;

    if (targets.empty()) return;

    auto dirs = fibonacciSphere((int)targets.size());

    std::lock_guard<std::mutex> lk(*ctx.physicsMutex);
    for (int i = 0; i < (int)targets.size(); ++i) {
        glm::vec3 imp = dirs[i] * explodeStrength;
        targets[i]->activate(true);
        targets[i]->applyCentralImpulse({ imp.x, imp.y, imp.z });
    }
    grabbed_.clear();
    selected_.clear();
}

void MoveAbility::update(float dt, const AbilityContext& ctx, bool qHeld)
{
    qHeld_   = qHeld;
    lmbHeld_ = ctx.lmbHeld;

    // Release grabbed bodies when LMB released
    if (!lmbHeld_ && !grabbed_.empty())
        grabbed_.clear();

    // Apply PID forces to grabbed bodies
    if (lmbHeld_ && !grabbed_.empty()) {
        std::lock_guard<std::mutex> lk(*ctx.physicsMutex);

        glm::vec3 targetPos = ctx.camPos + ctx.camFront * grabDist;

        for (auto& gb : grabbed_) 
		{
            btVector3 btBodyPos = gb.body->getWorldTransform().getOrigin();
            glm::vec3 bodyPos(btBodyPos.x(), btBodyPos.y(), btBodyPos.z());

            glm::vec3 error = targetPos - bodyPos;

            gb.integral  += error * dt;
            // glm::vec3 derivative = (dt > 0.f) ? (error - gb.prevError) / dt
            //                                    : glm::vec3(0.f);
            gb.prevError  = error;

			btVector3 velocity = gb.body->getLinearVelocity();
			glm::vec3 glm_velocity = {velocity[0], velocity[1], velocity[2]};
            glm::vec3 force = Kp * error + Ki * gb.integral - Kd * glm_velocity;

            float mag = std::sqrt(force.x*force.x + force.y*force.y + force.z*force.z);
            if (mag > maxForce && mag > 0.f)
                force *= (maxForce / mag);

            gb.body->activate(true);
            gb.body->applyCentralForce({ force.x, force.y, force.z });
        }
    }

    // Selection mode: collect bodies within kSelectionRadius of screen centre
    selected_.clear();
    if (qHeld && !lmbHeld_) {
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
            if (ndc.z > 1.f) continue;

            float screenX = (ndc.x * 0.5f + 0.5f) * (float)ctx.viewW;
            float screenY = (1.f - (ndc.y * 0.5f + 0.5f)) * (float)ctx.viewH;

            float dx = screenX - (float)ctx.viewW * 0.5f;
            float dy = screenY - (float)ctx.viewH * 0.5f;

            if (std::sqrt(dx * dx + dy * dy) <= kSelectionRadius)
                selected_.push_back(body);
        }
    }
}

void MoveAbility::onFire(const AbilityContext& ctx)
{
    if (selected_.empty()) return;

    grabbed_.clear();
    grabbed_.reserve(selected_.size());
    for (btRigidBody* body : selected_)
        grabbed_.push_back({ body, {}, {} });
    selected_.clear();
}

void MoveAbility::onDeselect()
{
    selected_.clear();
    grabbed_.clear();
    qHeld_   = false;
    lmbHeld_ = false;
}

void MoveAbility::onScroll(float delta)
{
    grabDist = std::clamp(grabDist + delta * 8.f, kMinDist, kMaxDist);
}

void MoveAbility::drawHUD(ImDrawList* dl, float cx, float cy)
{
	return;
    if (lmbHeld_ && !grabbed_.empty()) {
        // Grabbing — orange ring + count
        ImU32 col = IM_COL32(255, 160, 0, 220);
        dl->AddCircle({cx, cy}, kSelectionRadius, col, 32, 2.5f);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "MOVE [%d]", (int)grabbed_.size());
        dl->AddText({cx - 30.f, cy + kSelectionRadius + 5.f}, col, buf);
    } else if (qHeld_) {
        // Selection mode — grey or green ring
        ImU32 col = selected_.empty() ? IM_COL32(180, 180, 180, 150)
                                      : IM_COL32(50, 220, 50, 200);
        dl->AddCircle({cx, cy}, kSelectionRadius, col, 32, 2.f);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "x%d", (int)selected_.size());
        dl->AddText({cx - 10.f, cy + kSelectionRadius + 5.f}, col, buf);
    }
}

void MoveAbility::drawOverlay()
{
    ImGui::Separator();
    ImGui::Text("MOVE PID");
    ImGui::SliderFloat("Kp",        &Kp,       0.f,    100.f);
    ImGui::SliderFloat("Ki",        &Ki,       0.f,     10.f);
    ImGui::SliderFloat("Kd",        &Kd,       0.f,     50.f);
    ImGui::SliderFloat("Max Force", &maxForce, 0.f, 10000.f);
    ImGui::Text("Grab Dist: %.1f  (scroll to adjust)", grabDist);
    ImGui::SliderFloat("Explode Str", &explodeStrength, 0.f, 5000.f);
}
