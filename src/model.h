#pragma once
#include <string>
#include <vector>
#include "mesh.h"
#include "shader.h"

class Model
{
public:
    explicit Model(const std::string& path);
    explicit Model(std::vector<Mesh> meshes);

    void draw(Shader& shader) const;

private:
    std::vector<Mesh> meshes_;

    void loadAssimp(const std::string& path);
};
