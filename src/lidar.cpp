#define TINYCSOCKET_IMPLEMENTATION
#include "lidar.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>                                                                                                                                                                                
// #include <glm/gtx/quaternion.hpp>   

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

    tcs_lib_init();

    TcsResult rc = tcs_socket_preset(&socket_, TCS_PRESET_UDP_IP4);
    if (rc != TCS_SUCCESS)
    {
        fprintf(stderr, "LidarSystem: socket failed (%d)\n", rc);
        tcs_lib_free();
        return false;
    }

    struct TcsAddress addr = TCS_ADDRESS_NONE;
    addr.family           = TCS_AF_IP4;
    addr.data.ip4.address = TCS_ADDRESS_ANY_IP4;
    addr.data.ip4.port    = p;
    rc = tcs_bind(socket_, &addr);
    if (rc != TCS_SUCCESS)
    {
        fprintf(stderr, "LidarSystem: bind failed (%d)\n", rc);
        tcs_close(&socket_);
        tcs_lib_free();
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
    if (socket_ != TCS_SOCKET_INVALID)
    {
        tcs_close(&socket_);
        socket_ = TCS_SOCKET_INVALID;
        tcs_lib_free();
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

std::vector<LidarSystem::Frame> LidarSystem::drainFrames()
{
    std::lock_guard<std::mutex> lk(framesMutex_);
    std::vector<Frame> result(
        std::make_move_iterator(frames_.begin()),
        std::make_move_iterator(frames_.end()));
    frames_.clear();
    return result;
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

			float motor_rad = ((float)motor_angle)/((float)(1<<14));
			float motor_deg = motor_rad*180.f/M_PI;
			printf("%f\n", motor_deg);

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
    struct TcsPool* pool = nullptr;
    tcs_pool_create(&pool);
    tcs_pool_add(pool, socket_, nullptr, true, false, false);

    uint8_t buf[1242];
    while (running_)
    {
        struct TcsPollEvent ev = TCS_POOL_EVENT_EMPTY;
        size_t populated = 0;
        TcsResult prc = tcs_pool_poll(pool, &ev, 1, &populated, 200);
        if (prc == TCS_SUCCESS && populated > 0 && ev.can_read)
        {
            size_t received = 0;
            TcsResult rrc = tcs_receive(socket_, buf, sizeof(buf), TCS_FLAG_NONE, &received);
            if (rrc == TCS_SUCCESS && received == 1242)
                receivePacket(buf, 1242);
        }
    }

    tcs_pool_destroy(&pool);
}


static const glm::mat3 lidar_link_rot =                                                                                                                                                                                
	glm::mat3(glm::rotate(glm::mat4(1.f), (float)M_PI/2.f, {0,1,0})) *                                                                                                                                           
	glm::mat3(glm::rotate(glm::mat4(1.f), (float)M_PI/2.f, {0,0,1}));                                                                                                                                            
				

static const glm::quat lidar_link_quat =                                                                                                                                                                                
	glm::angleAxis((float)M_PI/2.f, glm::vec3(0,1,0)) *                                                                                                                                                          
	glm::angleAxis((float)M_PI/2.f, glm::vec3(0,0,1));   

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

		uint32_t raw = 0;
		raw = (uint32_t)(block[4]);
		raw |= (((uint32_t)(block[5])) << 8);
		raw |= (((uint32_t)(block[6])) << 16);
		int32_t motor_angle;
		if(raw & 0x800000)	//sign extend
		{
			motor_angle = raw | 0xFF000000;
		}
		else
		{
			motor_angle = raw;
		}

		float motor_rad = ((float)motor_angle)/((float)(1<<14));
		float motor_deg = motor_rad*180.f/M_PI;
		// glm::quat R = glm::angleAxis(motor_rad, glm::vec3(0,0,1)) * lidar_link_quat;
		glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.f), motor_rad, {0,0,1})) * lidar_link_rot;



		// printf("%f\n", motor_deg);

        for (int ch = 0; ch < 32; ++ch)
        {
            int            laser_id = ch % 16;
            const uint8_t* chan     = block + 7 + ch * 3;  // skip 3 motor_angle bytes
            uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
            if (dist_raw == 0) continue;

            float dist_m = dist_raw * 0.01f;
            float el_rad = VERT_ANGLES[laser_id] * static_cast<float>(M_PI) / 180.f;

			glm::vec3 point_lidar = glm::vec3(
                dist_m * cosf(el_rad) * sinf(az_rad),
                dist_m * cosf(el_rad) * cosf(az_rad),
                dist_m * sinf(el_rad)
            );

			glm::vec3 point_base = R*point_lidar;

            pending_.push_back(point_base);
        }

        lastAzimuth_ = azimuth_block;
    }

    // Commit every packet for minimum latency — ~380 points every ~1.3 ms
    commitPending();
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
