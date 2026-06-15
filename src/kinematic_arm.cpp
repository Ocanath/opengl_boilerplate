#include "kinematic_arm.h"
#include "shader.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

KinematicArm::KinematicArm(btDiscreteDynamicsWorld* world,
                             Model*                   cubeModel,
                             const std::vector<LinkDef>& defs,
                             glm::vec3                base)
    : base_(base), defs_(defs), cumAngles_(defs.size(), 0.f)
{
    links_.reserve(defs.size());
    for (const auto& d : defs) {
        links_.emplace_back(
            world, cubeModel,
            glm::vec3{d.length * 0.5f, d.width * 0.5f, d.depth * 0.5f},
            base,
            glm::vec3{d.length, d.width, d.depth},
            d.color,
            0.f,                                    // mass — kinematic bodies use 0
            glm::quat(1.f, 0.f, 0.f, 0.f),
            true);
    }
}

void KinematicArm::setThetas(const std::vector<float>& thetas)
{
    glm::vec3 joint    = base_;
    float     cumAngle = 0.f;

    for (size_t i = 0; i < links_.size(); ++i) {
        if (i < thetas.size())
            cumAngle += thetas[i];

        cumAngles_[i] = cumAngle;

        float     L      = defs_[i].length;
        glm::vec3 dir    = {std::cos(cumAngle), std::sin(cumAngle), 0.f};
        glm::vec3 center = joint + (L * 0.5f) * dir;
        glm::quat rot    = glm::angleAxis(cumAngle, glm::vec3{0.f, 0.f, 1.f});

        links_[i].syncKinematicPose(center, rot);
        joint = joint + L * dir;
    }
}

void KinematicArm::draw(Shader& shader) const
{
    for (const auto& link : links_)
        link.draw(shader);
}
