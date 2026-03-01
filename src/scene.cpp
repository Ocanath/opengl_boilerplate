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

    // Load the shared cube mesh used to visualise lights
    try {
        lightCubeModel_ = std::make_unique<Model>("assets/cube.obj");
    } catch (const std::exception& e) {
        fprintf(stderr, "Scene: could not load light cube: %s\n", e.what());
    }
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

    // Draw scene models
    shader.setInt("unlit", 0);
    for (auto& model : models_) {
        shader.setMat4("model", glm::mat4(1.f));
        shader.setVec3("objectColor", {0.7f, 0.7f, 0.75f});
        model.draw(shader);
    }

    // Draw one small emissive cube per light
    if (lightCubeModel_) {
        shader.setInt("unlit", 1);
        for (const auto& light : lights_) {
            glm::mat4 m = glm::scale(
                glm::translate(glm::mat4(1.f), light.position),
                glm::vec3(0.2f));
            shader.setMat4("model", m);
            shader.setVec3("objectColor", light.color);
            lightCubeModel_->draw(shader);
        }
        shader.setInt("unlit", 0);
    }
}
