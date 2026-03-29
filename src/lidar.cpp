#include "lidar.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// VLP-16 vertical angles (degrees) for lasers 0–15 (interleaved firing order)
static constexpr float VERT_ANGLES[16] = {
    -15.f, 1.f, -13.f, 3.f, -11.f, 5.f, -9.f, 7.f,
     -7.f, 9.f,  -5.f, 11.f, -3.f, 13.f, -1.f, 15.f
};

LidarSystem::~LidarSystem()
{
    disconnect();
}

bool LidarSystem::connect(uint16_t p)
{
    if (running_) disconnect();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        perror("LidarSystem: socket");
        return false;
    }

    // 200 ms receive timeout so recvLoop can check running_
    struct timeval tv { 0, 200000 };
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(p);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("LidarSystem: bind");
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    running_    = true;
    recvThread_ = std::thread(&LidarSystem::recvLoop, this);
    return true;
}

void LidarSystem::disconnect()
{
    running_ = false;
    if (recvThread_.joinable())
        recvThread_.join();
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
}

std::vector<LidarSystem::Frame> LidarSystem::getFrames() const
{
    std::lock_guard<std::mutex> lk(framesMutex_);
    return { frames_.begin(), frames_.end() };
}

bool LidarSystem::pollNewFrame()
{
    return newFrameFlag_.exchange(false);
}

// ── Decode a 1242-byte modified packet to LidarPoints ────────────────────────

std::vector<LidarSystem::LidarPoint> LidarSystem::decodeModifiedPacket(const uint8_t* data, size_t len)
{
    std::vector<LidarPoint> points;
    if (len < 1242) return points;

    for (int b = 0; b < 12; ++b)
    {
        const uint8_t* block = data + b * 103;
        if (block[0] != 0xFF || block[1] != 0xEE) continue;

        uint16_t az_raw      = static_cast<uint16_t>(block[2] | (block[3] << 8));
        int32_t  motor_angle = static_cast<int32_t>(
                                   static_cast<uint32_t>(block[4])
                                 | static_cast<uint32_t>(block[5]) << 8
                                 | static_cast<uint32_t>(block[6]) << 16);

        float az_rad = (az_raw / 100.f) * static_cast<float>(M_PI) / 180.f;

        for (int ch = 0; ch < 32; ++ch)
        {
            int            laser_id = ch % 16;
            const uint8_t* chan     = block + 7 + ch * 3;
            uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
            if (dist_raw == 0) continue;

            float dist_m = dist_raw * 0.01f;
            float el_rad = VERT_ANGLES[laser_id] * static_cast<float>(M_PI) / 180.f;

            LidarPoint lp;
            lp.pos = glm::vec3(
                dist_m * cosf(el_rad) * sinf(az_rad),
                dist_m * cosf(el_rad) * cosf(az_rad),
                dist_m * sinf(el_rad)
            );
            lp.motor_angle  = motor_angle;
            lp.azimuth_cdeg = az_raw;
            points.push_back(lp);
        }
    }
    return points;
}

// ── Receive thread ────────────────────────────────────────────────────────────

void LidarSystem::recvLoop()
{
    uint8_t buf[1242];
    while (running_)
    {
        ssize_t received = ::recvfrom(socket_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (received == 1242)
            receivePacket(buf, static_cast<size_t>(received));
        // On timeout (EAGAIN/EWOULDBLOCK), received < 0 — just loop and re-check running_
    }
}

// ── Modified packet receiver ──────────────────────────────────────────────────
//
// Modified packet layout (1242 bytes):
//   12 blocks × 103 bytes: [0xFF 0xEE][azimuth LE 2B][motor_angle LE 3B][32 channels × 3B]
//   Footer at byte 1236:   [timestamp LE 4B][return_mode 1B][model_id 1B]

void LidarSystem::receivePacket(const uint8_t* data, size_t /*len*/)
{
    // Footer timestamp (µs since top of hour)
    uint32_t ts = static_cast<uint32_t>(data[1236])
                | static_cast<uint32_t>(data[1237]) << 8
                | static_cast<uint32_t>(data[1238]) << 16
                | static_cast<uint32_t>(data[1239]) << 24;
    latestTimestamp_ = ts;

    for (int b = 0; b < 12; ++b)
    {
        const uint8_t* block = data + b * 103;
        if (block[0] != 0xFF || block[1] != 0xEE) continue;

        uint16_t az_raw        = static_cast<uint16_t>(block[2] | (block[3] << 8));
        float    azimuth_block = static_cast<float>(az_raw);
        float    az_rad        = (az_raw / 100.f) * static_cast<float>(M_PI) / 180.f;

        for (int ch = 0; ch < 32; ++ch)
        {
            int            laser_id = ch % 16;
            const uint8_t* chan     = block + 7 + ch * 3;  // skip 3 motor_angle bytes
            uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
            if (dist_raw == 0) continue;

            float dist_m = dist_raw * 0.01f;
            float el_rad = VERT_ANGLES[laser_id] * static_cast<float>(M_PI) / 180.f;

            pending_.push_back(glm::vec3(
                dist_m * cosf(el_rad) * sinf(az_rad),
                dist_m * cosf(el_rad) * cosf(az_rad),
                dist_m * sinf(el_rad)
            ));
        }

        if (lastAzimuth_ > 18000.f && azimuth_block < lastAzimuth_)
            commitPending();
        lastAzimuth_ = azimuth_block;
    }
}

void LidarSystem::commitPending()
{
    if (pending_.empty()) return;

    Frame frame;
    if (static_cast<int>(pending_.size()) <= maxPointsPerFrame)
    {
        frame = std::move(pending_);
    }
    else
    {
        frame.reserve(maxPointsPerFrame);
        float stride = static_cast<float>(pending_.size()) / static_cast<float>(maxPointsPerFrame);
        for (int i = 0; i < maxPointsPerFrame; ++i)
            frame.push_back(pending_[static_cast<size_t>(i * stride)]);
    }
    pending_.clear();

    {
        std::lock_guard<std::mutex> lk(framesMutex_);
        frames_.push_back(std::move(frame));
        while (static_cast<int>(frames_.size()) > bufferDepth)
            frames_.pop_front();
    }
    newFrameFlag_ = true;
}
