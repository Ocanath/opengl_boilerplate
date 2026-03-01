#pragma once
#include <glm/glm.hpp>

#define MAX_LIGHTS 16

struct Light {
    glm::vec3 position  = {0.f, 5.f, 0.f};
    glm::vec3 color     = {1.f, 1.f, 1.f};
    float     intensity = 1.f;
};
