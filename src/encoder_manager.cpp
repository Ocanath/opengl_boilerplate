#include "encoder_manager.h"
#include <chrono>

EncoderManager::EncoderManager()
{
    running_ = true;
    thread_  = std::thread(&EncoderManager::pollLoop, this);
}

EncoderManager::~EncoderManager()
{
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

std::vector<float> EncoderManager::getThetas() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return thetas_;
}

void EncoderManager::pollLoop()
{
    // TODO: set up Serial + Encoders here

    while (running_) {
        // TODO: read encoders, then:
        // std::lock_guard<std::mutex> lk(mutex_);
        // thetas_[i] = encoders[i]->theta;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
