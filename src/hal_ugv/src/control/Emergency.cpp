#include <hal_ugv/control/Emergency.h>

using namespace hal::ugv;

bool Emergency::SetGoal(
    hal_ugv::Emergency::Request  &req, 
    hal_ugv::Emergency::Response &res
) {
    // Eveything OK
    res.success = true;
    res.status  = "Successfully switched to Emergency controller";
    return true;
}

void Emergency::Update(const hal_ugv::State &state, 
    double dt, hal_ugv::Control &control)
{
	// Do something more elegant than this ultimately...
   	control.roll     = 0.0;
    control.pitch    = 0.0;
    control.yaw      = 0.0;
    control.throttle = 0.0;
}

// Goal reach implementations
bool Emergency::HasGoalBeenReached()
{
    return false;
}

// Reset implementations
void Emergency::Reset()
{
   // Do nothing
}
