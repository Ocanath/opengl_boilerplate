#include "mesh.h"
#include <glad/gl.h>
#include <utility>

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices)
    : vertices_(std::move(vertices)), indices_(std::move(indices))
{
    setup();
}

Mesh::~Mesh()
{
    if (VAO) { glDeleteVertexArrays(1, &VAO); }
    if (VBO) { glDeleteBuffers(1, &VBO); }
    if (EBO) { glDeleteBuffers(1, &EBO); }
}

Mesh::Mesh(Mesh&& other) noexcept
    : VAO(other.VAO), VBO(other.VBO), EBO(other.EBO),
      vertices_(std::move(other.vertices_)), indices_(std::move(other.indices_))
{
    other.VAO = other.VBO = other.EBO = 0;
}

void Mesh::setup()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(vertices_.size() * sizeof(Vertex)),
                 vertices_.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(indices_.size() * sizeof(unsigned int)),
                 indices_.data(),
                 GL_STATIC_DRAW);

    // position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, position));
    // normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, normal));
    // uv
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, uv));

    glBindVertexArray(0);
}

void Mesh::draw(Shader& shader) const
{
    (void)shader;
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES,
                   (GLsizei)indices_.size(),
                   GL_UNSIGNED_INT,
                   nullptr);
    glBindVertexArray(0);
}
