#include <hal_ugv/control/VelocityHeight.h>

// Controller constants
#define _X          0       
#define _Y          1
#define _Z          2
#define _YAW        3
#define _Kvp        0.25     /* xy velocity proportional constant          */
#define _Kvi        0.15     /* xy velocity integrative constant           */
#define _Kvd        0.1     /* xy velocity derivative constant            */
#define _Kiz        0.0008   /* altitude integrative constant              */
#define _Kpz        0.03     /* altitude proportional constant             */
#define _Kdz        0.04     /* altitude derivative constant               */
#define _th_hover   0.59     /* throttle hover offset                      */
#define _maxtilt    0.34     /* max pitch/roll angle                       */
#define _Kya        6.0      /* yaw proportional constant                  */
#define _maxyawrate 4.4      /* max allowed yaw rate                       */
#define _maxv       0.33      /* max allowed xy velocity                    */

using namespace hal::ugv;

// Incoming velocity and height control
bool VelocityHeight::SetGoal(
    hal_ugv::VelocityHeight::Request  &req, 
    hal_ugv::VelocityHeight::Response &res
) {
    // Horizontal velocity 
    sp[_X]      = req.dx;
    sp[_Y]      = req.dy;
    sp[_Z]      = req.z;
    sp[_YAW]    = req.yaw;

    // Try and switch control
    res.success = true;
    res.status  = "Successfully switched to VelocityHeight controller";
    return true;
}

// Update the control
void VelocityHeight::Update(const hal_ugv::State &state, 
    double dt, hal_ugv::Control &control)
{
    ///////////////// PID CONTROLLER FOR PITCH AND ROLL /////////////////////

    // Body-frame velocity (v) and b < n frame rotation (r)
    double vt[3], r[3];

    // Get the Euler orientation
    r[0] = state.roll;
    r[1] = state.pitch;
    r[2] = state.yaw;

    // Make a copy of the n-frame velocity
    vt[0] = sp[_X];
    vt[1] = sp[_Y];
    vt[2] = 0.0;

    // Get the n-frame (x,y) velocity in the b-frame
    n2b(r,vt);

    // Used for calculations below
    double e, de;

    // Pitch
    e =  (limit(vt[0],-_maxv,_maxv) - state.u);  
    de = (dt > 0 ? (e - ep[_X]) / dt : 0);
    ei[_X] += e * dt; 
    ep[_X]  = e;
    double desP = limit(_Kvp * e + _Kvi * ei[_X] + _Kvd * de,-_maxtilt,_maxtilt);

    // Roll
    e = -(limit(vt[1],-_maxv,_maxv) - state.v);     
    de = (dt > 0 ? (e - ep[_Y]) / dt : 0);
    ei[_Y] += e * dt; 
    ep[_Y]  = e;
    double desR = limit(_Kvp * e + _Kvi * ei[_Y] + _Kvd * de,-_maxtilt,_maxtilt);

    ////////////////////////// PID THROTTLE CONTROLLER //////////////////////////

    // Get the (P)roportional component
    double ez_ = sp[_Z] - state.z;

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
    
    //////////////////// P CONTROLLER FOR YAW //////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);

    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = th;
}

// Goal reach implementations
bool VelocityHeight::HasGoalBeenReached()
{
    return false;
}

// Reset implementations
void VelocityHeight::Reset()
{
    // For roll, pitch, height
    for (int i = 0; i < 3; i++)
    {
        ei[i] = 0.0;
        ep[i] = 0.0;
    }
}
