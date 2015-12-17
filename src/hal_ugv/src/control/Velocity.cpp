#include <hal_ugv/control/Velocity.h>

// Controller constants
#define _X          0       
#define _Y          1
#define _Z          2
#define _YAW        3
#define _Kvp        0.25     /* xy velocity proportional constant          */
#define _Kvi        0.15     /* xy velocity integrative constant           */
#define _Kvd        0.05     /* xy velocity derivative constant            */
#define _Kwp       -0.2      /* altitude integrative constant              */
#define _Kwi       -0.1      /* altitude proportional constant             */
#define _Kwd       -0.0      /* altitude derivative constant               */
#define _th_hover   0.59     /* throttle hover offset                      */
#define _maxtilt    0.34     /* max pitch/roll angle                       */
#define _Kya        6.0      /* yaw proportional constant                  */
#define _maxyawrate 4.4      /* max allowed yaw rate                       */
#define _maxv       3.0      /* max allowed xy velocity                    */

using namespace hal::ugv;

bool Velocity::SetGoal(
    hal_ugv::Velocity::Request  &req, 
    hal_ugv::Velocity::Response &res
) {
    // Set the velocity
    sp[_X]   = req.dx;
    sp[_Y]   = req.dy;
    sp[_Z]   = req.dz;
    sp[_YAW] = req.yaw;

    res.success = true;
    res.status  = "Successfully switched to Velocity controller";
    return true;
}

void Velocity::Update(const hal_ugv::State &state, 
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
    vt[2] = sp[_Z];

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
   
    //////////////////// PID CONTROLLER FOR THRUST //////////////////////

    // b-frame w
    e = -(limit(vt[_Z],-_maxv,_maxv) - state.w);
    de = (dt > 0 ? (e - ep[_Z]) / dt : 0);
    ei[_Z] += e * dt;
    ep[_Z] =  e;

    double desT = limit(_Kwp * e + _Kwi * ei[_Z] + _Kwd * de + _th_hover, 0.0, 1.0);

    //////////////////// P CONTROLLER FOR YAW //////////////////////

    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);

    //////////////////////// CONTROL PACKAGING /////////////////////////

    // This will be returned
    control.roll     = desR;
    control.pitch    = desP;
    control.yaw      = desY;
    control.throttle = desT;
}

// Goal reach implementations
bool Velocity::HasGoalBeenReached()
{
    return false;
}

// Reset implementations
void Velocity::Reset()
{
    for (int i = 0; i < 3; i++)
    {
        ei[i] = 0.0;
        ep[i] = 0.0;
    }
}
