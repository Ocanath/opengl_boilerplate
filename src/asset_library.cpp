#include "asset_library.h"
#include "model.h"
#include "mesh.h"

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

Model* getCachedMeshModel(const std::string& path)
{
    static std::unordered_map<std::string, std::unique_ptr<Model>> cache;
    auto it = cache.find(path);
    if (it != cache.end()) return it->second.get();
    return cache.emplace(path, std::make_unique<Model>(path)).first->second.get();
}

Model* getCachedBoxModel()
{
    return getCachedMeshModel("assets/cube.obj");
}

static Mesh buildCylinderMesh(int segments)
{
    constexpr float radius = 0.5f;
    constexpr float halfHeight = 0.5f;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    // Side wall: two rings sharing radial normals.
    unsigned int sideStart = 0;
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.f * kPi * (float)i / (float)segments;
        float x = std::cos(angle);
        float y = std::sin(angle);
        glm::vec3 normal(x, y, 0.f);

        vertices.push_back({ { radius * x, radius * y, -halfHeight }, normal, { 0.f, 0.f } });
        vertices.push_back({ { radius * x, radius * y,  halfHeight }, normal, { 0.f, 1.f } });
    }
    for (int i = 0; i < segments; ++i) {
        unsigned int b0 = sideStart + i * 2;
        unsigned int t0 = b0 + 1;
        unsigned int b1 = b0 + 2;
        unsigned int t1 = b0 + 3;
        indices.insert(indices.end(), { b0, b1, t0, t0, b1, t1 });
    }

    // Top cap (+Z).
    unsigned int topCenter = (unsigned int)vertices.size();
    vertices.push_back({ { 0.f, 0.f, halfHeight }, { 0.f, 0.f, 1.f }, { 0.5f, 0.5f } });
    unsigned int topRingStart = (unsigned int)vertices.size();
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.f * kPi * (float)i / (float)segments;
        float x = std::cos(angle), y = std::sin(angle);
        vertices.push_back({ { radius * x, radius * y, halfHeight }, { 0.f, 0.f, 1.f }, { 0.5f + 0.5f * x, 0.5f + 0.5f * y } });
    }
    for (int i = 0; i < segments; ++i) {
        indices.insert(indices.end(), { topCenter, topRingStart + i, topRingStart + i + 1 });
    }

    // Bottom cap (-Z).
    unsigned int botCenter = (unsigned int)vertices.size();
    vertices.push_back({ { 0.f, 0.f, -halfHeight }, { 0.f, 0.f, -1.f }, { 0.5f, 0.5f } });
    unsigned int botRingStart = (unsigned int)vertices.size();
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.f * kPi * (float)i / (float)segments;
        float x = std::cos(angle), y = std::sin(angle);
        vertices.push_back({ { radius * x, radius * y, -halfHeight }, { 0.f, 0.f, -1.f }, { 0.5f + 0.5f * x, 0.5f + 0.5f * y } });
    }
    for (int i = 0; i < segments; ++i) {
        indices.insert(indices.end(), { botCenter, botRingStart + i + 1, botRingStart + i });
    }

    return Mesh(std::move(vertices), std::move(indices));
}

static Mesh buildSphereMesh(int rings, int sectors)
{
    constexpr float radius = 0.5f;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    for (int r = 0; r <= rings; ++r) {
        float theta = kPi * (float)r / (float)rings; // 0 (top) .. pi (bottom)
        float y = std::cos(theta);
        float ringRadius = std::sin(theta);
        for (int s = 0; s <= sectors; ++s) {
            float phi = 2.f * kPi * (float)s / (float)sectors;
            float x = ringRadius * std::cos(phi);
            float z = ringRadius * std::sin(phi);
            glm::vec3 normal(x, y, z);
            vertices.push_back({ radius * normal, normal, { (float)s / sectors, (float)r / rings } });
        }
    }

    int stride = sectors + 1;
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            unsigned int a = r * stride + s;
            unsigned int b = a + stride;
            indices.insert(indices.end(), { a, b, a + 1, a + 1, b, b + 1 });
        }
    }

    return Mesh(std::move(vertices), std::move(indices));
}

Model* getCachedCylinderModel(int segments)
{
    static std::unordered_map<int, std::unique_ptr<Model>> cache;
    auto it = cache.find(segments);
    if (it != cache.end()) return it->second.get();

    std::vector<Mesh> meshes;
    meshes.push_back(buildCylinderMesh(segments));
    return cache.emplace(segments, std::make_unique<Model>(std::move(meshes))).first->second.get();
}

Model* getCachedSphereModel(int rings, int sectors)
{
    static std::unordered_map<long long, std::unique_ptr<Model>> cache;
    long long key = ((long long)rings << 32) | (unsigned int)sectors;
    auto it = cache.find(key);
    if (it != cache.end()) return it->second.get();

    std::vector<Mesh> meshes;
    meshes.push_back(buildSphereMesh(rings, sectors));
    return cache.emplace(key, std::make_unique<Model>(std::move(meshes))).first->second.get();
}
