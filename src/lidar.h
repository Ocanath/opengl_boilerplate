#pragma once
#include "tinycsocket.h"
#include <glm/glm.hpp>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>

class LidarSystem {
public:
    using Frame = std::vector<glm::vec3>;

    struct LidarPoint {
        glm::vec3 pos;
        int32_t   motor_angle;   // wrap_2pi_14b result restored from 3 packed bytes
        uint16_t  azimuth_cdeg;
    };

    LidarSystem() = default;
    ~LidarSystem();

    bool connect(uint16_t port);
    void disconnect();
    bool isConnected() const { return running_; }

    // Decode a 1242-byte modified packet to LidarPoints (includes motor_angle)
    static std::vector<LidarPoint> decodeModifiedPacket(const uint8_t* data, size_t len);

    // Thread-safe snapshot of all buffered frames (non-destructive)
    std::vector<Frame> getFrames() const;

    // Moves all buffered frames out of the internal deque and returns them.
    // More efficient than getFrames() when frames will only be consumed once.
    std::vector<Frame> drainFrames();

    // Returns true and consumes the flag if a new frame arrived since last call
    bool pollNewFrame();

    // Latest packet timestamp (microseconds since top of hour), 0 if none received
    uint32_t latestTimestamp() const { return latestTimestamp_.load(); }

    // Settings — set before connect()
    int      bufferDepth       = 1024;
    int      maxPointsPerFrame = 2000;
    uint16_t port              = 9000;

private:
    TcsSocket           socket_      = TCS_SOCKET_INVALID;
    std::thread         recvThread_;
    std::atomic<bool>   running_     = false;

    mutable std::mutex  framesMutex_;
    std::deque<Frame>   frames_;
    Frame               pending_;
    float               lastAzimuth_ = -1.f;
    std::atomic<bool>     newFrameFlag_    = false;
    std::atomic<uint32_t> latestTimestamp_ = 0;

    void recvLoop();
    void receivePacket(const uint8_t* data, size_t len);
    void commitPending();
};
