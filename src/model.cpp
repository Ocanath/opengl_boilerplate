#include "model.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <stdexcept>
#include <cstdio>

static Mesh processMesh(aiMesh* mesh)
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    vertices.reserve(mesh->mNumVertices);
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        Vertex v{};
        v.position = { mesh->mVertices[i].x,
                       mesh->mVertices[i].y,
                       mesh->mVertices[i].z };
        if (mesh->HasNormals()) {
            v.normal = { mesh->mNormals[i].x,
                         mesh->mNormals[i].y,
                         mesh->mNormals[i].z };
        }
        if (mesh->mTextureCoords[0]) {
            v.uv = { mesh->mTextureCoords[0][i].x,
                     mesh->mTextureCoords[0][i].y };
        }
        vertices.push_back(v);
    }

    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
        const aiFace& face = mesh->mFaces[f];
        for (unsigned int j = 0; j < face.mNumIndices; ++j)
            indices.push_back(face.mIndices[j]);
    }

    return Mesh(std::move(vertices), std::move(indices));
}

static void processNode(aiNode* node, const aiScene* scene,
                        std::vector<Mesh>& out)
{
    for (unsigned int i = 0; i < node->mNumMeshes; ++i)
        out.push_back(processMesh(scene->mMeshes[node->mMeshes[i]]));

    for (unsigned int i = 0; i < node->mNumChildren; ++i)
        processNode(node->mChildren[i], scene, out);
}

void Model::loadAssimp(const std::string& path)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        path,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_FlipUVs |
        aiProcess_CalcTangentSpace);

    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !scene->mRootNode) {
        throw std::runtime_error(std::string("Assimp: ") + importer.GetErrorString());
    }

    fprintf(stdout, "Model loaded: %s (%u meshes)\n",
            path.c_str(), scene->mNumMeshes);

    processNode(scene->mRootNode, scene, meshes_);
}

Model::Model(const std::string& path)
{
    loadAssimp(path);
}

void Model::draw(Shader& shader) const
{
    for (const Mesh& m : meshes_)
        m.draw(shader);
}
