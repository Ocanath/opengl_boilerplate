#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

class EncoderManager {
public:
    EncoderManager();
    ~EncoderManager();

    EncoderManager(const EncoderManager&)            = delete;
    EncoderManager& operator=(const EncoderManager&) = delete;

    std::vector<float> getThetas() const;

private:
    void pollLoop();

    mutable std::mutex mutex_;
    std::vector<float> thetas_;
    std::atomic<bool>  running_{false};
    std::thread        thread_;
};
