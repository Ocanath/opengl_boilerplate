#pragma once
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include "DynamicRobot.h"

class Shader;

// Owns everything puppet-related: the DynamicRobot simulation object, the
// serial encoder thread, the encoder-to-joint address mapping, and the PD
// control constants. Scene holds one of these and calls update() each frame
// while holding its physicsMutex_ — update() must NOT acquire physicsMutex_
// itself (it's already held by the caller).
class PuppetRobot {
public:
    explicit PuppetRobot(btDiscreteDynamicsWorld* world);
    ~PuppetRobot();

    PuppetRobot(const PuppetRobot&)            = delete;
    PuppetRobot& operator=(const PuppetRobot&) = delete;
    PuppetRobot(PuppetRobot&&)                 = delete;
    PuppetRobot& operator=(PuppetRobot&&)      = delete;

    // Read latest encoder angles and apply PD to each joint.
    // Must be called by the holder of Scene::physicsMutex_.
    void update();

    // Raw encoder readings indexed by encoder address (radians). Thread-safe.
    std::vector<float> getThetas() const;

    void renderCollision(Shader& shader) const;

private:
    void pollLoop();

    DynamicRobot       robot_;
    mutable std::mutex encoderMutex_;
    std::vector<float> thetas_;
    std::atomic<bool>  running_{false};
    std::thread        thread_;
};
