#include "scene.h"
#include "ability_beam.h"
#include "ability_gravity.h"
#include "ability_push.h"
#include "ability_move.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
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
    dynamicsWorld_->setGravity({ 0.f, 0.f, -9.8f });

    try {
        cubeModel_ = std::make_unique<Model>("assets/cube.obj");
    } catch (const std::exception& e) {
        fprintf(stderr, "Scene: could not load cube: %s\n", e.what());
    }

    camera_ = std::make_unique<Camera>(dynamicsWorld_);
    lidar_  = std::make_unique<LidarSystem>();

    buildChamber();
    // buildPillars();

    // Create deferred rendering shaders
    gShader_        = std::make_unique<Shader>("shaders/gbuffer.vert",  "shaders/gbuffer.frag");
    lightingShader_ = std::make_unique<Shader>("shaders/lighting.vert", "shaders/lighting.frag");
    unlitShader_    = std::make_unique<Shader>("shaders/unlit.vert",    "shaders/unlit.frag");

    // Bind G-buffer texture samplers (units 0,1,2) — set once, permanent
    lightingShader_->use();
    lightingShader_->setInt("gPosition", 0);
    lightingShader_->setInt("gNormal",   1);
    lightingShader_->setInt("gAlbedo",   2);

    // Point cloud shader + VAO/VBO
    pointCloudShader_ = std::make_unique<Shader>(
        "shaders/pointcloud.vert", "shaders/pointcloud.frag");
    glGenVertexArrays(1, &pointCloudVAO_);
    glGenBuffers(1, &pointCloudVBO_);
    glBindVertexArray(pointCloudVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, pointCloudVBO_);
    glBufferData(GL_ARRAY_BUFFER,
        (GLsizeiptr)(MAX_GPU_POINTS * sizeof(glm::vec3)), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glBindVertexArray(0);

    // Light SSBO
    glGenBuffers(1, &lightSSBO_);

    // Fullscreen quad
    initQuad();

    // G-buffer is sized on first draw (viewport size unknown at construct time)

    // Register abilities
    abilities_.push_back(std::make_unique<BeamAbility>(cubeModel_.get()));
    abilities_.push_back(std::make_unique<GravitySwitchAbility>());
    abilities_.push_back(std::make_unique<PushAbility>());
    abilities_.push_back(std::make_unique<MoveAbility>());

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
    lidar_.reset();           // stop recv thread before touching physics
    abilities_.clear();       // BeamAbility's firedBeams_ removed from physics world
    lightBoxes_.clear();
    floatingPillars_.clear();
    chamberWalls_.clear();
    camera_.reset();

    delete dynamicsWorld_;
    delete solver_;
    delete dispatcher_;
    delete collConfig_;
    delete broadphase_;

    // Destroy deferred rendering GL objects
    destroyGBuffer();
    glDeleteBuffers(1,      &lightSSBO_);
    glDeleteVertexArrays(1, &quadVAO_);
    glDeleteBuffers(1,      &quadVBO_);
    glDeleteVertexArrays(1, &pointCloudVAO_);
    glDeleteBuffers(1,      &pointCloudVBO_);
}

// ── G-buffer helpers ──────────────────────────────────────────────────────────

void Scene::initGBuffer(int w, int h)
{
    destroyGBuffer();

    glGenFramebuffers(1, &gFBO_);
    glBindFramebuffer(GL_FRAMEBUFFER, gFBO_);

    auto makeColorTex = [&](unsigned int& tex, GLenum internalFmt) {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFmt, w, h, 0,
                     GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    };

    makeColorTex(gPos_,    GL_RGBA16F);
    makeColorTex(gNorm_,   GL_RGBA16F);
    makeColorTex(gAlbedo_, GL_RGBA8);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gPos_,    0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, gNorm_,   0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, gAlbedo_, 0);

    // Depth+stencil renderbuffer
    glGenRenderbuffers(1, &gDepthRBO_);
    glBindRenderbuffer(GL_RENDERBUFFER, gDepthRBO_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, gDepthRBO_);

    GLenum bufs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
    glDrawBuffers(3, bufs);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        fprintf(stderr, "Scene: G-buffer FBO incomplete!\n");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    gWidth_  = w;
    gHeight_ = h;
}

void Scene::destroyGBuffer()
{
    if (gFBO_)      { glDeleteFramebuffers(1,  &gFBO_);      gFBO_      = 0; }
    if (gPos_)      { glDeleteTextures(1,      &gPos_);      gPos_      = 0; }
    if (gNorm_)     { glDeleteTextures(1,      &gNorm_);     gNorm_     = 0; }
    if (gAlbedo_)   { glDeleteTextures(1,      &gAlbedo_);   gAlbedo_   = 0; }
    if (gDepthRBO_) { glDeleteRenderbuffers(1, &gDepthRBO_); gDepthRBO_ = 0; }
    gWidth_ = gHeight_ = 0;
}

void Scene::initQuad()
{
    // NDC fullscreen quad: two triangles covering [-1,1]×[-1,1]
    // Layout: vec2 pos, vec2 uv — interleaved
    static const float verts[] = {
        // pos        // uv
        -1.f,  1.f,   0.f, 1.f,
        -1.f, -1.f,   0.f, 0.f,
         1.f, -1.f,   1.f, 0.f,

        -1.f,  1.f,   0.f, 1.f,
         1.f, -1.f,   1.f, 0.f,
         1.f,  1.f,   1.f, 1.f,
    };

    glGenVertexArrays(1, &quadVAO_);
    glGenBuffers(1, &quadVBO_);
    glBindVertexArray(quadVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    // aPos (location 0) — vec2
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          reinterpret_cast<void*>(0));
    // aUV  (location 1) — vec2
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          reinterpret_cast<void*>(2 * sizeof(float)));

    glBindVertexArray(0);
}

void Scene::uploadLights()
{
    if (lights_.empty()) return;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, lightSSBO_);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 (GLsizeiptr)(lights_.size() * sizeof(Light)),
                 lights_.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, lightSSBO_);
}

// ── Physics loop ──────────────────────────────────────────────────────────────

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

// ── Scene construction helpers ────────────────────────────────────────────────

void Scene::buildChamber()
{
    if (!cubeModel_) return;

    struct WallDef { glm::vec3 pos, half, scale; };
    static const WallDef walls[] = {
        // Floor   (Z = -0.5)
        {{ 0.f,   0.f,  -0.5f}, {50.f, 50.f, 0.5f}, {100.f, 100.f,   1.f}},
        // Ceiling (Z = 100.5)
        {{ 0.f,   0.f, 100.5f}, {50.f, 50.f, 0.5f}, {100.f, 100.f,   1.f}},
        // -X wall
        {{-50.5f, 0.f,  50.f},  {0.5f, 50.f, 50.f}, {  1.f, 100.f, 100.f}},
        // +X wall
        {{ 50.5f, 0.f,  50.f},  {0.5f, 50.f, 50.f}, {  1.f, 100.f, 100.f}},
        // -Y wall
        {{ 0.f, -50.5f, 50.f},  {50.f, 0.5f, 50.f}, {100.f,   1.f, 100.f}},
        // +Y wall
        {{ 0.f,  50.5f, 50.f},  {50.f, 0.5f, 50.f}, {100.f,   1.f, 100.f}},
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

    for (float xpos = -30; xpos < 30; xpos += 6)
    {
        for (float ypos = -30; ypos < 30; ypos += 6)
        {
            for (float zpos = 30.f; zpos < 30+30.f; zpos += 6.f)
            {
                glm::vec3 pos = { xpos, ypos, zpos };
                floatingPillars_.emplace_back(
                    dynamicsWorld_, cubeModel_.get(),
                    glm::vec3{0.2f, 0.2f, 3.f/2.f},
                    pos,
                    glm::vec3{0.4f, 0.4f, 3.f},
                    glm::vec3{0.039f, 0.039f, 0.039f},
                    4.f);  // dynamic mass

                btRigidBody* b = floatingPillars_.back().getBody();
                b->setGravity({ 0.f, 0.f, 0.f });
                b->setDamping(0.0f, 0.0f);
                b->setActivationState(DISABLE_DEACTIVATION);
                b->setAngularVelocity({xpos/10, ypos/10, zpos/10});
            }
        }
    }
}

// ── Public interface ──────────────────────────────────────────────────────────

void Scene::addModel(const std::string& path)
{
    models_.emplace_back(path);
}

void Scene::addLight(const Light& light)
{
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
        b->setDamping(0.01f, 0.01f);
    }
}

glm::vec3 Scene::getCameraPosition()
{
    std::lock_guard<std::mutex> lk(physicsMutex_);
    return camera_->getPosition();
}

void Scene::setCameraGravity(bool enabled)
{
    std::lock_guard<std::mutex> lk(physicsMutex_);
    camera_->setGravity(enabled);
}

void Scene::saveCameraToFile(const std::string& path)
{
    std::lock_guard<std::mutex> lk(physicsMutex_);
    camera_->saveToFile(path);
}

void Scene::loadCameraFromFile(const std::string& path)
{
    std::lock_guard<std::mutex> lk(physicsMutex_);
    camera_->loadFromFile(path);
}

void Scene::selectAbility(int i)
{
    if (i == activeAbility_ || i < 0 || i >= (int)abilities_.size()) return;
    abilities_[activeAbility_]->onDeselect();
    activeAbility_ = i;
}

void Scene::firePrimary()
{
    if (!camera_->mouseCaptured || abilities_.empty()) return;
    AbilityContext ctx {
        dynamicsWorld_, &physicsMutex_, cubeModel_.get(),
        camPos_, camFront_, lastView_, lastProj_,
        lastViewW_, lastViewH_, false, lights_
    };
    abilities_[activeAbility_]->onFire(ctx);
}

void Scene::fireSecondary()
{
    if (!camera_->mouseCaptured || abilities_.empty()) return;
    AbilityContext ctx {
        dynamicsWorld_, &physicsMutex_, cubeModel_.get(),
        camPos_, camFront_, lastView_, lastProj_,
        lastViewW_, lastViewH_, false, lights_
    };
    abilities_[activeAbility_]->onFireSecondary(ctx);
}

const char* Scene::getAbilityName(int i) const
{
    if (i < 0 || i >= (int)abilities_.size()) return "";
    return abilities_[i]->name();
}

float& Scene::beamFireVelocity()
{
    return static_cast<BeamAbility*>(abilities_[0].get())->fireVelocity;
}

void Scene::drawActiveAbilityHUD(ImDrawList* dl, float cx, float cy)
{
    if (!abilities_.empty())
        abilities_[activeAbility_]->drawHUD(dl, cx, cy);
}

void Scene::onScroll(float delta)
{
    if (!abilities_.empty())
        abilities_[activeAbility_]->onScroll(delta);
}

void Scene::drawActiveAbilityOverlay()
{
    if (!abilities_.empty())
        abilities_[activeAbility_]->drawOverlay();
}

void Scene::update(float dt, GLFWwindow* window)
{
    camera_->processKeyboard(window);

    {
        std::lock_guard<std::mutex> lk(physicsMutex_);
        camera_->applyVelocity();
        for (size_t i = 0; i < lights_.size() && i < lightBoxes_.size(); ++i)
            lights_[i].position = lightBoxes_[i].getPosition();

        // Snapshot camera state for ability system
        camPos_   = camera_->getPosition();
        camFront_ = camera_->getFront();
    }

    // Update all abilities; only active one gets qHeld=true.
    // Inactive abilities still tick so that timed effects (drag, gravity) complete.
    if (!abilities_.empty()) {
        bool lmbHeld = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
                       && camera_->mouseCaptured;
        AbilityContext ctx {
            dynamicsWorld_, &physicsMutex_, cubeModel_.get(),
            camPos_, camFront_, lastView_, lastProj_,
            lastViewW_, lastViewH_, lmbHeld, lights_
        };
        // bool qHeld = (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) && camera_->mouseCaptured;
		bool qHeld = true;
        for (int i = 0; i < (int)abilities_.size(); ++i)
		{
			abilities_[i]->update(dt, ctx, i == activeAbility_ ? qHeld : false);
		}
    }

    if (lidar_->pollNewFrame())
        updateLidarPoints();
}

// ── LiDAR point cloud ─────────────────────────────────────────────────────────

void Scene::updateLidarPoints()
{
    auto frames = lidar_->drainFrames();
    if (frames.empty()) return;

    glBindBuffer(GL_ARRAY_BUFFER, pointCloudVBO_);
    for (auto& frame : frames)
    {
        int n = (int)frame.size();
        if (n == 0) continue;

        int spaceToEnd = MAX_GPU_POINTS - gpuWriteHead_;
        if (n <= spaceToEnd)
        {
            glBufferSubData(GL_ARRAY_BUFFER,
                (GLintptr)(gpuWriteHead_ * (int)sizeof(glm::vec3)),
                (GLsizeiptr)(n * (int)sizeof(glm::vec3)),
                frame.data());
        }
        else
        {
            // Frame straddles the wrap point — split into two uploads
            glBufferSubData(GL_ARRAY_BUFFER,
                (GLintptr)(gpuWriteHead_ * (int)sizeof(glm::vec3)),
                (GLsizeiptr)(spaceToEnd * (int)sizeof(glm::vec3)),
                frame.data());
            glBufferSubData(GL_ARRAY_BUFFER,
                0,
                (GLsizeiptr)((n - spaceToEnd) * (int)sizeof(glm::vec3)),
                frame.data() + spaceToEnd);
        }
        gpuWriteHead_ = (gpuWriteHead_ + n) % MAX_GPU_POINTS;
        gpuTotalPts_  = std::min(gpuTotalPts_ + n, MAX_GPU_POINTS);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ── 3-pass deferred draw ──────────────────────────────────────────────────────

void Scene::draw(int width, int height)
{
    if (width <= 0 || height <= 0) return;

    // Rebuild G-buffer if viewport changed
    if (width != gWidth_ || height != gHeight_)
        initGBuffer(width, height);

    // Upload lights to SSBO
    uploadLights();

    // Snapshot camera state under lock
    glm::mat4 view;
    glm::vec3 camPos;
    {
        std::lock_guard<std::mutex> lk(physicsMutex_);
        view   = camera_->getViewMatrix();
        camPos = camera_->getPosition();
    }

    float     aspect = (float)width / (float)height;
    glm::mat4 proj   = glm::perspective(glm::radians(60.f), aspect, 0.1f, 1000.f);

    // Store for ability system (gravity selection projection, etc.)
    lastView_  = view;
    lastProj_  = proj;
    lastViewW_ = width;
    lastViewH_ = height;

    // ── Pass 1: Geometry → G-buffer ──────────────────────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, gFBO_);
    glViewport(0, 0, width, height);
    glClearColor(0.f, 0.f, 0.f, 0.f); // w=0 marks sky in lighting pass
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    gShader_->use();
    gShader_->setMat4("view",       view);
    gShader_->setMat4("projection", proj);

    // Scene models
    for (auto& model : models_) {
        gShader_->setMat4("model",       glm::mat4(1.f));
        gShader_->setVec3("objectColor", {0.7f, 0.7f, 0.75f});
        model.draw(*gShader_);
    }

    // Chamber walls
    for (auto& wall : chamberWalls_)
        wall.draw(*gShader_);

    // Floating pillars
    for (auto& pillar : floatingPillars_)
        pillar.draw(*gShader_);


    // ── Pass 2: Lighting → default FBO ───────────────────────────────────────
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width, height);
    glClearColor(0.08f, 0.08f, 0.12f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    lightingShader_->use();
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, gPos_);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, gNorm_);
    glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, gAlbedo_);
    lightingShader_->setVec3("viewPos",   camPos);
    lightingShader_->setInt ("numLights", (int)lights_.size());

    glBindVertexArray(quadVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    // ── Pass 3: Blit depth + forward unlit ───────────────────────────────────
    glBindFramebuffer(GL_READ_FRAMEBUFFER, gFBO_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height,
                      0, 0, width, height,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glEnable(GL_DEPTH_TEST);

    unlitShader_->use();
    unlitShader_->setMat4("view",       view);
    unlitShader_->setMat4("projection", proj);

    for (auto& lb : lightBoxes_)
        lb.draw(*unlitShader_);

    // Ability preview (active only) + owned boxes from every ability
    if (!abilities_.empty()) {
        abilities_[activeAbility_]->drawPreview(*unlitShader_, view, proj);
        for (auto& ab : abilities_)
            if (auto* boxes = ab->getBoxes())
                for (const auto& box : *boxes)
                    box.draw(*unlitShader_);
    }

    // LiDAR point cloud — GPU ring buffer, one draw call
    if (gpuTotalPts_ > 0) {
        glEnable(GL_PROGRAM_POINT_SIZE);
        pointCloudShader_->use();
        pointCloudShader_->setMat4("view",       view);
        pointCloudShader_->setMat4("projection", proj);
        pointCloudShader_->setFloat("pointSize", 4.f);
        pointCloudShader_->setVec3("color",      {1.f, 0.f, 0.f});
        glBindVertexArray(pointCloudVAO_);
        glDrawArrays(GL_POINTS, 0, gpuTotalPts_);
        glBindVertexArray(0);
        glDisable(GL_PROGRAM_POINT_SIZE);
    }
}



void Scene::addPile(glm::vec3 loc)
{
	for(float z = 1.f; z <= 3.f; z+=1.f)
	{
		for(float y = -1.f; y <= 1.f; y+=1.f)
		{
			for(float x = -1.f; x <= 1.f; x+=1.f)
			{
				floatingPillars_.emplace_back(                    
					dynamicsWorld_, cubeModel_.get(),
                    glm::vec3{0.5f, 0.5f, 0.5},
                    glm::vec3{x,y,z} + loc,
                    glm::vec3{1.f, 1.f, 1.f},
                    glm::vec3{0.039f, 0.039f, 0.039f},
                    1.f);  // dynamic mass
				
				btRigidBody* b = floatingPillars_.back().getBody();
                b->setGravity({ 0.f, 0.f, -9.8f });
                b->setDamping(0.0f, 0.0f);
                // b->setActivationState(DISABLE_DEACTIVATION);
			}
		}
	}

}