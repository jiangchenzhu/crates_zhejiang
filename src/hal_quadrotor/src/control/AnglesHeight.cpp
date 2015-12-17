#include <hal_quadrotor/control/AnglesHeight.h>

// Constant parameters
#define _Kxy        0.9         /* position proportional constant */
#define _Kv         0.09        /* velocity proportional constant */
#define _Kiz        0.0008      /* altitude integrative constant  */
#define _Kpz        0.03        /* altitude proportional constant */ 
#define _Kdz        0.04        /* altitude derivative constant   */      
#define _th_hover   0.59        /* throttle hover offset          */
#define _maxtilt    0.34        /* max pitch/roll angle (23deg)   */
#define _Kya        6.0         /* yaw proportional constant      */
#define _maxyawrate 4.4         /* max allowed yaw rate (200dps)  */
#define _maxv       3.0         /* max allowed xy velocity        */

using namespace hal::quadrotor;

bool AnglesHeight::SetGoal(
    hal_quadrotor::AnglesHeight::Request  &req, 
    hal_quadrotor::AnglesHeight::Response &res
) {
    // Set the new goal state from the message
    sp[_ROLL]   = req.roll;
    sp[_PITCH]  = req.pitch;
    sp[_YAW]    = req.yaw;
    sp[_HEIGHT] = req.z;

    // Eveything OK
    res.success = true;
    res.status  = "Successfully switched to AnglesHeight controller";
    return true;
}

void AnglesHeight::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
    ////////////////////////// YAW CONTROLLER /////////////////////////

    double ya = limit(_Kya*(sp[_YAW] - state.yaw),-_maxyawrate,_maxyawrate);

    //////////////////////// THROTTLE CONTROLLER ////////////////////////

    // Get the (P)roportional component (sign change by Andrew)
    double ez_ = sp[_HEIGHT] - state.z;

    // Get the (I)ntegral component
    iz += ez_ * dt;
    
    // Get the (D)erivative component
    double de_ = (dt > 0 ? (ez_ - ez) / dt : 0);
    double desth = _th_hover + _Kpz * ez_ + _Kiz * iz + de_ * _Kdz;
    double th = limit(desth,0.0,1.0);
    
    // Save (P) contribution for next derivative calculation
    ez = ez_;

    // Save (I) contribution for next derivative calculation
    iz = iz - (desth - th) * 2.0;
    
    //////////////////////// CONTROL PACKAGING /////////////////////////
    
    // This will be returned
    control.roll     = -sp[_ROLL];
    control.pitch    =  sp[_PITCH];
    control.yaw      =  ya;
    control.throttle =  th;
}

// Goal reach implementations
bool AnglesHeight::HasGoalBeenReached()
{
    return false;
}

// Reset implementations
void AnglesHeight::Reset()
{
    iz = 0.0;
    ez = 0.0;
}