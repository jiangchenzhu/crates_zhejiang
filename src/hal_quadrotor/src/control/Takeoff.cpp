#include <hal_quadrotor/control/Takeoff.h>

// COntroller constants
#define _X          0
#define _Y          1
#define _Z          2
#define _YAW        3
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

#define _goal_dist  2.0         /* distance considered reached    */

using namespace hal::quadrotor;

bool Takeoff::SetGoal(
    hal_quadrotor::Takeoff::Request  &req, 
    hal_quadrotor::Takeoff::Response &res
) {
    // Set the new goal
    sp[_Z] = req.altitude;

    // Have we reached our desination
    reach = false;

    // Try and switch control
    res.success = true;
    res.status  = "Successfully switched to Takeoff controller";
    return true;
}
#include <ros/ros.h>
void Takeoff::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
    /////////////////// CHECK IF GOAL IS REACHED /////////////////// 

    //ROS_INFO("%f",dt);

	if (dt > 0)
    {
        double dst = sqrt(
                (sp[_X]-state.x)*(sp[_X]-state.x) 
            +   (sp[_Y]-state.y)*(sp[_Y]-state.y)
            +   (sp[_Z]-state.z)*(sp[_Z]-state.z)
        );
        if (!reach && dst < _goal_dist)
            reach = true;
    }
    else
	{
	    sp[_X]   = state.x;
	    sp[_Y]   = state.y;
	    sp[_YAW] = state.yaw;
	}
    
    ////////////////////// CALCULATE VELOCITIES ////////////////////

    double vel[3] = {sp[_X]-state.x, sp[_Y]-state.y, sp[_Z]-state.z};
    double rot[3] = {state.roll, state.pitch, state.yaw};
    n2b(rot,vel);
    double bx = vel[0];
    double by = vel[1];

    //////////////////////// P ROLL CONTROLLER ////////////////////////

    double desv = limit(_Kxy*by,-_maxv,_maxv);
    double desR = limit(-_Kv*(desv - state.v), -_maxtilt, _maxtilt);

    //////////////////////// P PITCH CONTROLLER ////////////////////////

    double desu = limit(_Kxy*bx,-_maxv,_maxv);
    double desP = limit( _Kv*(desu - state.u), -_maxtilt, _maxtilt);

    //////////////////////// P YAW CONTROLLER ////////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);
    
    /////////////////// PID THROTTLE CONTROLLER //////////////////////

    // Get the (P)roportional component (Changed by Andrew)
    double ez_ = sp[_Z] - state.z;

    // Get the (I)ntegral component
    iz += ez_ * dt;
    
    // Get the (D)erivative component
    double de_ = (dt > 0 ? (ez_ - ez) / dt : 0);
    double desth = _th_hover + _Kpz * ez_ + _Kiz * iz + de_ * _Kdz;
    double th = limit(desth,0,1);
    
    // Save (P) contribution for next derivative calculation
    ez = ez_;

    // Save (I) contribution for next derivative calculation
    iz = iz - (desth - th) * 2.0;

    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = th;
}

// Goal reach implementations
bool Takeoff::HasGoalBeenReached()
{
    return reach;
}

// Reset implementations
void Takeoff::Reset()
{
    iz = 0.0;
    ez = 0.0;
}