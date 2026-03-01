#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "shader.h"

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
};

class Mesh
{
public:
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices);
    ~Mesh();

    // Non-copyable (owns GPU resources)
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&&) = delete;

    void draw(Shader& shader) const;

private:
    unsigned int VAO = 0, VBO = 0, EBO = 0;
    std::vector<Vertex>       vertices_;
    std::vector<unsigned int> indices_;

    void setup();
};
