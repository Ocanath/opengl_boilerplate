#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "scene.h"
#include "light.h"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

// ─── Window dimensions ───────────────────────────────────────────────────────
static constexpr int WIN_W = 1280;
static constexpr int WIN_H = 720;

// ─── Global scene pointer for GLFW callbacks ─────────────────────────────────
static Scene*      g_scene   = nullptr;
static GLFWwindow* g_window  = nullptr;
static bool        g_firstMouse = true;
static double      g_lastX = WIN_W / 2.0, g_lastY = WIN_H / 2.0;

// ─── GLFW callbacks ──────────────────────────────────────────────────────────
static void framebufferSizeCallback(GLFWwindow*, int w, int h)
{
    glViewport(0, 0, w, h);
}

static void cursorPosCallback(GLFWwindow*, double xpos, double ypos)
{
    if (!g_scene) return;
    Camera& cam = g_scene->getCamera();
    if (!cam.mouseCaptured) {
        g_firstMouse = true;
        return;
    }

    if (g_firstMouse) {
        g_lastX = xpos;
        g_lastY = ypos;
        g_firstMouse = false;
    }

    double dx = xpos - g_lastX;
    double dy = ypos - g_lastY;
    g_lastX = xpos;
    g_lastY = ypos;

    cam.processMouse(dx, dy);
}

static void mouseButtonCallback(GLFWwindow*, int btn, int action, int /*mods*/)
{
    if (action != GLFW_PRESS || !g_scene || ImGui::GetIO().WantCaptureMouse) return;
    if (btn == GLFW_MOUSE_BUTTON_LEFT)
        g_scene->firePrimary();
    else if (btn == GLFW_MOUSE_BUTTON_RIGHT)
        g_scene->fireSecondary();
}

static void keyCallback(GLFWwindow* window, int key, int /*scancode*/,
                        int action, int /*mods*/)
{
    if (!g_scene) return;
    Camera& cam = g_scene->getCamera();

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        if (cam.mouseCaptured) {
            cam.mouseCaptured = false;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        } else {
            cam.mouseCaptured = true;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            g_firstMouse = true;
        }
    }

    if (key == GLFW_KEY_1 && action == GLFW_PRESS) g_scene->selectAbility(0);
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) g_scene->selectAbility(1);
    if (key == GLFW_KEY_3 && action == GLFW_PRESS) g_scene->selectAbility(2);
}

// ─── GLAD error callback ─────────────────────────────────────────────────────
static void APIENTRY glDebugCallback(GLenum /*source*/, GLenum type,
                                     GLuint /*id*/, GLenum severity,
                                     GLsizei /*length*/, const GLchar* message,
                                     const void* /*userParam*/)
{
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    fprintf(stderr, "[GL] %s: %s\n",
            (type == GL_DEBUG_TYPE_ERROR ? "ERROR" : "INFO"), message);
}

// ─── Main ────────────────────────────────────────────────────────────────────
int main()
{
    // GLFW init
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialise GLFW\n");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifndef NDEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif

    g_window = glfwCreateWindow(WIN_W, WIN_H, "OpenGL Boilerplate", nullptr, nullptr);
    if (!g_window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(0); // uncapped framerate

    // GLAD
    if (!gladLoadGL(glfwGetProcAddress)) {
        fprintf(stderr, "Failed to initialise GLAD\n");
        return EXIT_FAILURE;
    }
    fprintf(stdout, "OpenGL %s\n", glGetString(GL_VERSION));

#ifndef NDEBUG
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(glDebugCallback, nullptr);
#endif

    glViewport(0, 0, WIN_W, WIN_H);
    glEnable(GL_DEPTH_TEST);

    // Callbacks
    glfwSetFramebufferSizeCallback(g_window, framebufferSizeCallback);
    glfwSetCursorPosCallback(g_window, cursorPosCallback);
    glfwSetKeyCallback(g_window, keyCallback);
    glfwSetMouseButtonCallback(g_window, mouseButtonCallback);
    glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(g_window, true);
    ImGui_ImplOpenGL3_Init("#version 460");

    // Scene — wrapped in a scope so destructors run before glfwDestroyWindow.
    {
    Scene scene;
    g_scene = &scene;
    scene.loadCameraFromFile("camera.position");

    // Add some default lights: { position, intensity, color, radius }
    scene.addLight({ {  5.f,  5.f, 18.f}, 15.f, {1.f, 0.95f, 0.85f}, 100.f });
    // scene.addLight({ { 5.f, 6.f, 5.f}, 15.f, {0.7f, 0.8f,  1.f},  100.f });
    // scene.addLight({ {  0.f, 3.f,  0.f}, 10.f, {1.f, 1.f,    1.f}, 100.f });

    // Load the default unit cube as the test mesh
    // scene.addModel("assets/cube.obj");

    // Timing
    double prevTime = glfwGetTime();

    // ─── Main loop ────────────────────────────────────────────────────────
    while (!glfwWindowShouldClose(g_window))
    {
        glfwPollEvents();

        double nowTime = glfwGetTime();
        float  dt      = (float)(nowTime - prevTime);
        prevTime       = nowTime;
        dt = (dt > 0.1f) ? 0.1f : dt; // clamp large deltas

        // Update
        scene.update(dt, g_window);

        // Render (deferred — scene owns all shaders now)
        int fbW, fbH;
        glfwGetFramebufferSize(g_window, &fbW, &fbH);
        scene.draw(fbW, fbH);

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Overlay — moveable and collapsible
        ImGui::SetNextWindowPos({10.f, 10.f}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({340.f, 0.f}, ImGuiCond_FirstUseEver);
        ImGui::Begin("Overlay");

        ImGui::Text("FPS: %.1f  (%.2f ms)", 1.f / dt, dt * 1000.f);
        glm::vec3 pos = scene.getCameraPosition();
        Camera& cam = scene.getCamera();
        ImGui::Text("Camera: (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
        ImGui::Text("Mouse: %s  [Esc toggles]", cam.mouseCaptured ? "captured" : "free");

        bool gravOn = cam.gravityEnabled;
        if (ImGui::Button(gravOn ? "Disable Gravity" : "Enable Gravity"))
		{
			scene.setCameraGravity(!gravOn);
		}

        ImGui::SameLine();
        if (ImGui::Button(cam.freecam ? "Freecam: ON" : "Freecam: OFF"))
            cam.freecam = !cam.freecam;

        ImGui::SliderFloat("XY Speed", &cam.moveSpeed,  0.5f, 50.f);
        ImGui::SliderFloat("Z Speed",  &cam.zMoveSpeed, 0.5f, 50.f);

        if (ImGui::Button("Save Camera"))
            scene.saveCameraToFile("camera.position");

        ImGui::Separator();
        auto& lights = scene.getLights();
        ImGui::Text("Lights: %d", (int)lights.size());

        for (int i = 0; i < (int)lights.size(); ++i) {
            ImGui::PushID(i);
            char label[32];
            snprintf(label, sizeof(label), "Light %d", i);
            if (ImGui::TreeNode(label)) {
                ImGui::DragFloat3("Position",  &lights[i].position.x, 0.1f);
                ImGui::ColorEdit3("Color",     &lights[i].color.x);
                ImGui::SliderFloat("Intensity", &lights[i].intensity, 0.f, 100.f);
                ImGui::SliderFloat("Radius",    &lights[i].radius,    1.f, 500.f);
                ImGui::TreePop();
            }
            ImGui::PopID();
        }

        if (ImGui::Button("Add Light")) {
            scene.addLight({ {0.f, 0.f, 20.f}, 1.f, {1.f, 1.f, 1.f}, 100.f });
        }

        ImGui::Separator();
        ImGui::Text("Abilities  [1/2/3]");
        for (int i = 0; i < scene.getAbilityCount(); ++i) {
            bool active = (i == scene.getActiveAbility());
            if (active) ImGui::PushStyleColor(ImGuiCol_Button, {0.3f, 0.6f, 1.f, 1.f});
            if (ImGui::Button(scene.getAbilityName(i))) scene.selectAbility(i);
            if (active) ImGui::PopStyleColor();
            ImGui::SameLine();
        }
        ImGui::NewLine();
        ImGui::Text("Q = activate  LMB = fire");
        if (scene.getAbilityCount() > 0)
            ImGui::SliderFloat("Beam Velocity", &scene.beamFireVelocity(), 1.f, 1000.f);

        ImGui::End();

        // Foreground HUD overlays (drawn over everything, outside any window)
        {
            float cx = (float)fbW * 0.5f;
            float cy = (float)fbH * 0.5f;
            auto* dl = ImGui::GetForegroundDrawList();
            dl->AddCircleFilled({cx, cy}, 3.f, IM_COL32(255, 255, 255, 200));
            scene.drawActiveAbilityHUD(dl, cx, cy);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(g_window);
    }

    g_scene = nullptr;
    } // scene destroyed here — GL context still live

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(g_window);
    glfwTerminate();

    return EXIT_SUCCESS;
}
