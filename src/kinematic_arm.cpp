#include "kinematic_arm.h"
#include "shader.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

KinematicArm::KinematicArm(btDiscreteDynamicsWorld* world,
                             Model*                   cubeModel,
                             const std::vector<LinkDef>& defs,
                             glm::vec3                base,
                             float                    motorGain,
                             float                    maxImpulse)
    : world_(world), defs_(defs), motorGain_(motorGain), maxImpulse_(maxImpulse)
{
    // Dynamic links laid out along +X from base (zero-angle configuration)
    links_.reserve(defs.size());
    float xOffset = 0.f;
    for (const auto& d : defs) {
        glm::vec3 center = base + glm::vec3(xOffset + d.length * 0.5f, 0.f, 0.f);
        xOffset += d.length;
        links_.emplace_back(
            world_, cubeModel,
            glm::vec3{d.length * 0.5f, d.width * 0.5f, d.depth * 0.5f},
            center,
            glm::vec3{d.length, d.width, d.depth},
            d.color,
            d.mass,
            glm::quat(1.f, 0.f, 0.f, 0.f),
            false);
        links_.back().getBody()->setDamping(0.3f, 0.5f);
        links_.back().getBody()->setActivationState(DISABLE_DEACTIVATION);
    }

    // One hinge per adjacent link pair — no world anchor
    for (size_t i = 0; i + 1 < links_.size(); ++i) {
        float Li     = defs[i].length;
        float Linext = defs[i + 1].length;
        auto* h = new btHingeConstraint(
            *links_[i].getBody(), *links_[i + 1].getBody(),
            btVector3( Li     * 0.5f, 0.f, 0.f),   // right end of link i
            btVector3(-Linext * 0.5f, 0.f, 0.f),   // left  end of link i+1
            btVector3(0.f, 0.f, 1.f),
            btVector3(0.f, 0.f, 1.f));
        h->enableMotor(true);
        h->setMaxMotorImpulse(maxImpulse_);
        world_->addConstraint(h, true);
        hinges_.push_back(h);
    }
}

KinematicArm::~KinematicArm()
{
    for (auto* h : hinges_) {
        world_->removeConstraint(h);
        delete h;
    }
    // links_ CollisionBox destructors remove link bodies from world
}

void KinematicArm::setThetas(const std::vector<float>& thetas)
{
    for (size_t i = 0; i < hinges_.size(); ++i) {
        if (i >= thetas.size()) {
            // No target for this joint — disable motor, let it flop free
            hinges_[i]->enableAngularMotor(false, 0.f, 0.f);
            continue;
        }
        float target  = thetas[i];
        float current = hinges_[i]->getHingeAngle();
        float error   = target - current;
        // Wrap to [-pi, pi]
        while (error >  (float)M_PI) error -= 2.f * (float)M_PI;
        while (error < -(float)M_PI) error += 2.f * (float)M_PI;
        hinges_[i]->enableAngularMotor(true, error * motorGain_, maxImpulse_);
    }
}

void KinematicArm::draw(Shader& shader) const
{
    for (const auto& link : links_)
        link.draw(shader);
}
