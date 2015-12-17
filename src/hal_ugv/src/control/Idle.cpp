#include <hal_ugv/control/Idle.h>

using namespace hal::ugv;

void Idle::Update(const hal_ugv::State &state, 
    double dt, hal_ugv::Control &control)
{
    control.roll     = 0.0;
    control.pitch    = 0.0;
    control.yaw      = 0.0;
    control.throttle = 0.0;
}

// Goal reach implementations
bool Idle::HasGoalBeenReached()
{
    return false;
}

// Reset implementations
void Idle::Reset()
{
   // Do nothing
}
