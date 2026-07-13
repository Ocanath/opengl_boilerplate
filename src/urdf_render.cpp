#include "urdf_render.h"
#include "asset_library.h"
#include "model.h"
#include "shader.h"

#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cstdio>
#include <stdexcept>

namespace {

// Debug color for every <collision> primitive — collisions carry no
// material, so there's nothing else to color them by.
constexpr glm::vec3 kCollisionColor{0.2f, 0.9f, 0.3f};
constexpr glm::vec3 kDefaultVisualColor{0.7f, 0.7f, 0.75f}; // <visual> with no <material><color>

glm::mat4 toGlmMat4(const btTransform& t)
{
    glm::mat4 m;
    t.getOpenGLMatrix(glm::value_ptr(m));
    return m;
}

// box/cylinder/sphere -> a cached asset_library primitive, scaled to match.
// Never called with GeometryType::Mesh (callers branch on that separately).
Model* resolvePrimitiveModel(const urdf::Geometry& geom, glm::vec3& outScale)
{
    switch (geom.type) {
        case urdf::GeometryType::Box:
            outScale = glm::vec3((float)geom.boxSize.x, (float)geom.boxSize.y, (float)geom.boxSize.z);
            return getCachedBoxModel();
        case urdf::GeometryType::Cylinder:
            outScale = glm::vec3((float)(2.0 * geom.cylinderRadius), (float)(2.0 * geom.cylinderRadius), (float)geom.cylinderLength);
            return getCachedCylinderModel();
        case urdf::GeometryType::Sphere: {
            float d = (float)(2.0 * geom.sphereRadius);
            outScale = glm::vec3(d, d, d);
            return getCachedSphereModel();
        }
        case urdf::GeometryType::Mesh:
            return nullptr;
    }
    return nullptr;
}

// Every distinct path that fails to load only warns once, no matter how
// many links/instances reference it. This runs at model-load time, not per
// frame, so a linear scan over what's typically a handful of broken paths
// is simpler than a lookup table.
bool alreadyWarnedAbout(const std::string& path)
{
    static std::vector<std::string> warned;
    for (const std::string& p : warned)
        if (p == path) return true;
    warned.push_back(path);
    return false;
}

Model* resolveMeshModel(const std::string& resolvedPath)
{
    try {
        return getCachedMeshModel(resolvedPath);
    } catch (const std::exception& e) {
        if (!alreadyWarnedAbout(resolvedPath)) {
            fprintf(stderr,
                    "urdf_render: warning: could not load visual mesh \"%s\": %s "
                    "(this link will render with no visual)\n",
                    resolvedPath.c_str(), e.what());
        }
        return nullptr;
    }
}

} // namespace

UrdfRender::UrdfRender(const urdf::BuildResult& build, const std::string& meshBaseDir)
{
    for (const urdf::VisualInstance& v : build.visuals) {
        Model*    model = nullptr;
        glm::vec3 scale{1.f};

        if (v.geometry.type == urdf::GeometryType::Mesh) {
            model = resolveMeshModel(meshBaseDir + v.geometry.meshFilename);
            scale = glm::vec3((float)v.geometry.meshScale.x, (float)v.geometry.meshScale.y, (float)v.geometry.meshScale.z);
        } else {
            model = resolvePrimitiveModel(v.geometry, scale);
        }
        if (!model) continue; // load failure already warned about; just skip drawing it

        Instance inst;
        inst.body = v.body;
        inst.model = model;
        inst.localTransform = toGlmMat4(v.localTransform) * glm::scale(glm::mat4(1.f), scale);
        inst.color = v.material.hasColor ? glm::vec3(v.material.r, v.material.g, v.material.b) : kDefaultVisualColor;
        visualInstances_.push_back(inst);
    }

    for (const urdf::CollisionInstance& c : build.collisions) {
        glm::vec3 scale{1.f};
        Model* model = resolvePrimitiveModel(c.geometry, scale); // parser rejects mesh <collision>, always a primitive here

        Instance inst;
        inst.body = c.body;
        inst.model = model;
        inst.localTransform = toGlmMat4(c.localTransform) * glm::scale(glm::mat4(1.f), scale);
        inst.color = kCollisionColor;
        collisionInstances_.push_back(inst);
    }
}

void UrdfRender::drawVisual(Shader& shader) const
{
    for (const Instance& inst : visualInstances_) {
        shader.setMat4("model", toGlmMat4(inst.body->getWorldTransform()) * inst.localTransform);
        shader.setVec3("objectColor", inst.color);
        inst.model->draw(shader);
    }
}

void UrdfRender::drawCollision(Shader& shader) const
{
    for (const Instance& inst : collisionInstances_) {
        shader.setMat4("model", toGlmMat4(inst.body->getWorldTransform()) * inst.localTransform);
        shader.setVec3("objectColor", inst.color);
        inst.model->draw(shader);
    }
}
