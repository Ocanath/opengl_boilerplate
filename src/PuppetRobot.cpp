#include "PuppetRobot.h"
#include "encoder.h"
#include <cstdio>

// Physical encoder address → URDF joint index. Identity to start as a
// wiring sanity check; update once the real hardware mapping is known.
// (See "Puppet joint index map" printed at startup for joint names.)
static const std::pair<int, int> kEncoderToJointIndex[] = {
    {0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4},
    {5, 5}, {6, 6}, {7, 7}, {8, 8}, {9, 9},
};

static constexpr float kKp         = 20.f;
static constexpr float kKd         = 0.5f;
static constexpr float kMaxImpulse = 3000.f;

PuppetRobot::PuppetRobot(btDiscreteDynamicsWorld* world)
    : robot_("assets/puppet.urdf", btVector3(0, -50, 5), "", 10.f, 20.0, 0.3, 60)
{
    robot_.buildBulletRobot(world);

    printf("Puppet joint index map:\n");
    for (size_t i = 0; i < robot_.getJointCount(); i++)
        printf("  [%zu] %s\n", i, robot_.getJointName(i).c_str());

    thetas_.resize(10, 0.f);
    running_ = true;
    thread_  = std::thread(&PuppetRobot::pollLoop, this);
}

PuppetRobot::~PuppetRobot()
{
    running_ = false;
    if (thread_.joinable())
        thread_.join();
    // ~DynamicRobot() removes its bodies from the world automatically.
}

void PuppetRobot::update()
{
    std::vector<float> thetas;
    {
        std::lock_guard<std::mutex> lk(encoderMutex_);
        thetas = thetas_;
    }

    for (const auto& [encoderAddr, jointIndex] : kEncoderToJointIndex) {
        if (encoderAddr < 0 || (size_t)encoderAddr >= thetas.size()) continue;
        robot_.setJointTargetAngle((size_t)jointIndex, thetas[encoderAddr], kKp, kKd, kMaxImpulse);
    }
}

std::vector<float> PuppetRobot::getThetas() const
{
    std::lock_guard<std::mutex> lk(encoderMutex_);
    return thetas_;
}

void PuppetRobot::renderCollision(Shader& shader) const
{
    robot_.renderCollision(shader);
}

void PuppetRobot::pollLoop()
{
    Serial ser;
    ser.autoconnect(921600);
    std::vector<Encoder*> encoders;
    for (int i = 0; i < 10; i++)
        encoders.push_back(new Encoder((unsigned char)i, &ser));

    while (running_) {
        for (int i = 0; i < (int)encoders.size(); i++) {
            int rc = encoders[i]->read_angle();
            (void)rc; // failures silently skipped; hardware is polled continuously
            std::lock_guard<std::mutex> lk(encoderMutex_);
            thetas_[i] = -encoders[i]->theta;
        }
    }

    for (auto* e : encoders)
        delete e;
}
