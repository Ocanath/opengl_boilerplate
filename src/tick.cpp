#include "tick.h"
#include <chrono>

using Clock = std::chrono::steady_clock;
static Clock::time_point g_start;

void init_tick()
{
    g_start = Clock::now();
}

int64_t get_tick_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now() - g_start).count();
}

int64_t get_tick_us()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               Clock::now() - g_start).count();
}
