#include "scene.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cstdio>

Scene::Scene()
{
    broadphase_  = new btDbvtBroadphase();
    collConfig_  = new btDefaultCollisionConfiguration();
    dispatcher_  = new btCollisionDispatcher(collConfig_);
    solver_      = new btSequentialImpulseConstraintSolver();
    dynamicsWorld_ = new btDiscreteDynamicsWorld(
        dispatcher_, broadphase_, solver_, collConfig_);
    dynamicsWorld_->setGravity({ 0.f, -9.81f, 0.f });

    camera_ = std::make_unique<Camera>(dynamicsWorld_);
}

Scene::~Scene()
{
    camera_.reset();

    if (groundBody_) {
        dynamicsWorld_->removeRigidBody(groundBody_);
        delete groundBody_->getMotionState();
        delete groundBody_->getCollisionShape();
        delete groundBody_;
    }

    delete dynamicsWorld_;
    delete solver_;
    delete dispatcher_;
    delete collConfig_;
    delete broadphase_;
}

void Scene::addModel(const std::string& path)
{
    models_.emplace_back(path);
}

void Scene::addLight(const Light& light)
{
    if ((int)lights_.size() < MAX_LIGHTS)
        lights_.push_back(light);
    else
        fprintf(stderr, "Scene: MAX_LIGHTS reached, ignoring extra light\n");
}

void Scene::addGroundPlane(float y)
{
    auto* shape = new btStaticPlaneShape({ 0, 1, 0 }, y);
    btTransform t;
    t.setIdentity();
    auto* ms = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo ci(0.f, ms, shape);
    groundBody_ = new btRigidBody(ci);
    dynamicsWorld_->addRigidBody(groundBody_);
}

void Scene::update(float dt, GLFWwindow* window)
{
    camera_->processKeyboard(window);
    dynamicsWorld_->stepSimulation(dt, 10);
    camera_->update(dt);
}

void Scene::draw(Shader& shader, int width, int height)
{
    shader.use();

    // Upload camera uniforms
    glm::mat4 view = camera_->getViewMatrix();
    float aspect = (height > 0) ? (float)width / (float)height : 1.f;
    glm::mat4 proj = glm::perspective(glm::radians(60.f), aspect, 0.1f, 1000.f);

    shader.setMat4("view",       view);
    shader.setMat4("projection", proj);
    shader.setVec3("viewPos",    camera_->getPosition());

    // Upload lights
    int n = (int)std::min(lights_.size(), (size_t)MAX_LIGHTS);
    shader.setInt("numLights", n);
    for (int i = 0; i < n; ++i) {
        shader.setVec3i ("lightPositions",  i, lights_[i].position);
        shader.setVec3i ("lightColors",     i, lights_[i].color);
        shader.setFloati("lightIntensities", i, lights_[i].intensity);
    }

    // Draw each model with identity transform
    for (auto& model : models_) {
        glm::mat4 modelMat = glm::mat4(1.f);
        shader.setMat4("model", modelMat);
        shader.setVec3("objectColor", {0.7f, 0.7f, 0.75f});
        model.draw(shader);
    }
}
