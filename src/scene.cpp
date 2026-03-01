#include "scene.h"
#include <btBulletDynamicsCommon.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Scene::Scene()
{
    broadphase_    = new btDbvtBroadphase();
    collConfig_    = new btDefaultCollisionConfiguration();
    dispatcher_    = new btCollisionDispatcher(collConfig_);
    solver_        = new btSequentialImpulseConstraintSolver();
    dynamicsWorld_ = new btDiscreteDynamicsWorld(
        dispatcher_, broadphase_, solver_, collConfig_);
    dynamicsWorld_->setGravity({ 0.f, -9.81f, 0.f });

    try {
        cubeModel_ = std::make_unique<Model>("assets/cube.obj");
    } catch (const std::exception& e) {
        fprintf(stderr, "Scene: could not load cube: %s\n", e.what());
    }

    camera_ = std::make_unique<Camera>(dynamicsWorld_);

    buildChamber();
    buildPillars();

    // Start dedicated physics thread (~120 Hz)
    physicsRunning_ = true;
    physicsThread_  = std::thread(&Scene::physicsLoop, this);
}

Scene::~Scene()
{
    // Stop physics thread before touching the world
    physicsRunning_ = false;
    if (physicsThread_.joinable())
        physicsThread_.join();

    // Remove collision bodies from world (in reverse dependency order)
    lightBoxes_.clear();
    floatingPillars_.clear();
    chamberWalls_.clear();
    camera_.reset();

    delete dynamicsWorld_;
    delete solver_;
    delete dispatcher_;
    delete collConfig_;
    delete broadphase_;
}

void Scene::physicsLoop()
{
    using Clock = std::chrono::steady_clock;
    auto prev = Clock::now();

    while (physicsRunning_) {
        auto  now = Clock::now();
        float dt  = std::min(
            std::chrono::duration<float>(now - prev).count(), 0.05f);
        prev = now;

        {
            std::lock_guard<std::mutex> lk(physicsMutex_);
            dynamicsWorld_->stepSimulation(dt, 10, 1.f / 120.f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}

void Scene::buildChamber()
{
    if (!cubeModel_) return;

    struct WallDef { glm::vec3 pos, half, scale; };
    static const WallDef walls[] = {
        // Floor
        {{ 0.f,   -0.5f,  0.f}, {50.f, 0.5f, 50.f}, {100.f,   1.f, 100.f}},
        // Ceiling
        {{ 0.f,  100.5f,  0.f}, {50.f, 0.5f, 50.f}, {100.f,   1.f, 100.f}},
        // -X wall
        {{-50.5f,  50.f,  0.f}, {0.5f, 50.f, 50.f}, {  1.f, 100.f, 100.f}},
        // +X wall
        {{ 50.5f,  50.f,  0.f}, {0.5f, 50.f, 50.f}, {  1.f, 100.f, 100.f}},
        // -Z wall
        {{ 0.f,    50.f, -50.5f}, {50.f, 50.f, 0.5f}, {100.f, 100.f,   1.f}},
        // +Z wall
        {{ 0.f,    50.f,  50.5f}, {50.f, 50.f, 0.5f}, {100.f, 100.f,   1.f}},
    };

    for (const auto& w : walls) {
        chamberWalls_.emplace_back(
            dynamicsWorld_, cubeModel_.get(),
            w.half, w.pos, w.scale,
            glm::vec3{0.18f, 0.18f, 0.18f});
    }
}

void Scene::buildPillars()
{
    if (!cubeModel_) return;

    // for (int i = 0; i < 5; ++i) 
	for(float xpos = -20; xpos < 20; xpos += 3)
	{
		for(float ypos = -20; ypos < 20; ypos += 3)
		{
			for(float zpos = 4.f; zpos < 20.f; zpos += 6.f)
			{
				// float     a   = i * 2.f * (float)M_PI / 5.f;
				glm::vec3 pos = { xpos, zpos, ypos };
				floatingPillars_.emplace_back(
					dynamicsWorld_, cubeModel_.get(),
					glm::vec3{0.1f, 3.f/2.f, 0.1f},
					pos,
					glm::vec3{0.2f, 3.f, 0.2f},
					glm::vec3{0.039f, 0.039f, 0.039f},
					1.f);  // dynamic mass
				
				// Float in place: disable gravity per-body, add heavy damping
				btRigidBody* b = floatingPillars_.back().getBody();
				b->setGravity({ 0.f, 0.f, 0.f });
				b->setDamping(0.0f, 0.0f);
				b->setAngularVelocity({0.0,0.1,0.0});

			}
		}
    }
}

void Scene::addModel(const std::string& path)
{
    models_.emplace_back(path);
}

void Scene::addLight(const Light& light)
{
    if ((int)lights_.size() >= MAX_LIGHTS) {
        fprintf(stderr, "Scene: MAX_LIGHTS reached, ignoring extra light\n");
        return;
    }
    lights_.push_back(light);

    if (cubeModel_) {
        std::lock_guard<std::mutex> lk(physicsMutex_);
        lightBoxes_.emplace_back(
            dynamicsWorld_, cubeModel_.get(),
            glm::vec3{0.1f, 0.1f, 3.1f},
            light.position,
            glm::vec3{0.2f, 0.2f, 0.2f},
            light.color,
            1.f,
            glm::quat(1.f, 0.f, 0.f, 0.f),
            false); // dynamic
        btRigidBody* b = lightBoxes_.back().getBody();
        b->setGravity({ 0.f, 0.f, 0.f });
        b->setDamping(0.1f, 0.1f);
    }
}

glm::vec3 Scene::getCameraPosition()
{
    std::lock_guard<std::mutex> lk(physicsMutex_);
    return camera_->getPosition();
}

void Scene::update(float /*dt*/, GLFWwindow* window)
{
    camera_->processKeyboard(window); // CPU only, no mutex

    {
        std::lock_guard<std::mutex> lk(physicsMutex_);
        camera_->applyVelocity();
        for (size_t i = 0; i < lights_.size() && i < lightBoxes_.size(); ++i)
            lights_[i].position = lightBoxes_[i].getPosition();
    }
}

void Scene::draw(Shader& shader, int width, int height)
{
    shader.use();

    // Snapshot camera state under lock, then release before GL calls
    glm::mat4 view;
    glm::vec3 camPos;
    {
        std::lock_guard<std::mutex> lk(physicsMutex_);
        view   = camera_->getViewMatrix();
        camPos = camera_->getPosition();
    }

    float aspect = (height > 0) ? (float)width / (float)height : 1.f;
    glm::mat4 proj = glm::perspective(glm::radians(60.f), aspect, 0.1f, 1000.f);

    shader.setMat4("view",       view);
    shader.setMat4("projection", proj);
    shader.setVec3("viewPos",    camPos);

    // Upload lights
    int n = (int)std::min(lights_.size(), (size_t)MAX_LIGHTS);
    shader.setInt("numLights", n);
    for (int i = 0; i < n; ++i) {
        shader.setVec3i ("lightPositions",   i, lights_[i].position);
        shader.setVec3i ("lightColors",      i, lights_[i].color);
        shader.setFloati("lightIntensities", i, lights_[i].intensity);
    }

    // Scene models (lit)
    shader.setInt("unlit", 0);
    for (auto& model : models_) {
        shader.setMat4("model",       glm::mat4(1.f));
        shader.setVec3("objectColor", {0.7f, 0.7f, 0.75f});
        model.draw(shader);
    }

    // Chamber walls (lit, dark grey)
    shader.setInt("unlit", 0);
    for (auto& wall : chamberWalls_)
        wall.draw(shader);

    // Floating pillars (lit, near-black)
    shader.setInt("unlit", 0);
    for (auto& pillar : floatingPillars_)
        pillar.draw(shader);

    // Light cubes (unlit / emissive) — rendered via physics transform for rotation
    if (!lightBoxes_.empty()) {
        shader.setInt("unlit", 1);
        for (auto& lb : lightBoxes_)
            lb.draw(shader);
        shader.setInt("unlit", 0);
    }
}
