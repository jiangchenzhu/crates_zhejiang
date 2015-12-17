#include <hal_quadrotor/control/Land.h>

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

using namespace hal::quadrotor;

bool Land::SetGoal(
    hal_quadrotor::Land::Request  &req, 
    hal_quadrotor::Land::Response &res
) {
    // Have we reached our desination
    reach = false;

    // Try and switch control
    res.success = true;
    res.status  = "Successfully switched to Land controller";
    return true;
}
#include <ros/ros.h>
void Land::Update(const hal_quadrotor::State &state, 
    double dt, hal_quadrotor::Control &control)
{
    ////////////////////// CHECK IF GOAL IS REACHED //////////////////////

    if (dt > 0)
    {   
        double landing_speed;           // m/s

        if (state.z > 2.0) 
            landing_speed = 1.0;
        else 
            landing_speed = 0.2;
        sp[_Z] -= dt*landing_speed;


        if (state.z > sp[_Z] + 2.0 && !stopped)
        {
            stopped = true;
            t_stopped = state.t;
        }
        if (state.z <= sp[_Z] + 2.0) stopped = false;

        // Check if landed
        if (stopped && state.t > t_stopped + 3.0 && state.z < 1.0 ) reach = true;

        ROS_INFO("Distance above land WP : %f",state.z - sp[_Z]);
    }
    else
    {
        sp[_X]   = state.x;
        sp[_Y]   = state.y;
        sp[_Z]   = state.z;
        sp[_YAW] = state.yaw;
    }

    ////////////////////// CALCULATE VELOCITIES ////////////////////

    double vel[3] = {sp[_X]-state.x, sp[_Y]-state.y, sp[_Z]-state.z};
    double rot[3] = {state.roll, state.pitch, state.yaw};
    n2b(rot,vel);
    double bx = vel[0];
    double by = vel[1];

    // Roll controller
    double desv = limit(_Kxy*by,-_maxv,_maxv);
    double desR = limit(-_Kv*(desv - state.v), -_maxtilt, _maxtilt);

    // Pitch controller
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
bool Land::HasGoalBeenReached()
{
    return reach;
}

// Reset implementations
void Land::Reset()
{
    iz = 0.0;
    ez = 0.0;
}