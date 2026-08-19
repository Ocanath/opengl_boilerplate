#include "encoder_manager.h"
#include "encoder.h"
#include <chrono>

EncoderManager::EncoderManager()
{
    thetas_.resize(10, 0.f);   // one slot per encoder, addresses 0-9
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
	Serial ser;
	ser.autoconnect(921600);
	std::vector<Encoder *> enc_arm;
	for(int i = 0; i < 10; i++)
	{
		enc_arm.push_back(new Encoder((unsigned char)i, &ser));
	}

    while (running_)
	{
		for(int i = 0; i < (int)enc_arm.size(); i++)
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

	for(int i = 0; i < (int)enc_arm.size(); i++)
	{
		delete enc_arm[i];
	}
}
