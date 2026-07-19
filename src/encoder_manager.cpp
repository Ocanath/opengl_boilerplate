#include "encoder_manager.h"
#include "encoder.h"
#include <chrono>

EncoderManager::EncoderManager()
{
    thetas_.resize(2, 0.f);   // one slot per encoder
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
	Serial ser;
	ser.autoconnect(921600);
	std::vector<Encoder *> enc_arm;
	Encoder e1(0, &ser);
	Encoder e2(1, &ser);
	enc_arm.push_back(&e1);
	enc_arm.push_back(&e2);
	
    while (running_) 
	{
        // TODO: read encoders, then:
        // std::lock_guard<std::mutex> lk(mutex_);
        // thetas_[i] = encoders[i]->theta;
		for(int i = 0; i < enc_arm.size(); i++)
		{
			int rc = enc_arm[i]->read_angle();
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				// printf("Failed to read encoder %d: code %d\n", i, rc);
			}
			std::lock_guard<std::mutex> lk (mutex_);
			thetas_[i] = enc_arm[i]->theta;
		}
		// printf("[%f, %f]\n", enc_arm[0]->theta, enc_arm[1]->theta);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
