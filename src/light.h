#pragma once
#include <glm/glm.hpp>

struct Light {
    glm::vec3 position  = {0.f, 5.f, 0.f};
    float     intensity = 1.f;               // packs with position → 16 bytes
    glm::vec3 color     = {1.f, 1.f, 1.f};
    float     radius    = 100.f;             // packs with color → 16 bytes
};
// Total: 32 bytes, matches GLSL std430 LightData exactly — no padding needed
