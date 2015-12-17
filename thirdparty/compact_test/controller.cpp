// Standard includes
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

// For converting WGS84 <-> LTP coordinates
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

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
#define _maxtilt    0.34        /* max pitch/roll angle           */
#define _Kya        6.0         /* yaw proportional constant      */
#define _maxyawrate 4.4         /* max allowed yaw rate           */
#define _maxv       5.0         /* max allowed xy velocity        */

// State
struct STATE
{
	double x;
	double y;
	double z;
	double pitch;
	double roll;
	double yaw;
	double u;
	double v;
};

double limit(const double& val, const double& minval, const double& maxval)
{
    if (val < minval) return minval;
    if (val > maxval) return maxval;
    return val;
}

void n2b(double rot[3], double vec[3])
{
    double t[3], c[3], s[3];
    for (int i = 0; i < 3; i++)
    {
        t[i] = vec[i];
        c[i] = cos(rot[i]);
        s[i] = sin(rot[i]);
    }
    vec[0] =                (c[1]*c[2])*t[0] +                (c[1]*s[2])*t[1] -      s[1]*t[2];
    vec[1] = (s[1]*s[0]*c[2]-s[2]*c[0])*t[0] + (s[1]*s[0]*s[2]+c[2]*c[0])*t[1] + c[1]*s[0]*t[2];
    vec[2] = (s[1]*c[0]*c[2]-s[2]*s[0])*t[0] + (s[1]*c[0]*s[2]+c[2]*s[0])*t[1] + c[1]*c[0]*t[2];
}

// Main entry point
int main(int argc, char* argv[])
{	
	double sp[] = {0, 1, 2, 4};
	STATE  state;
	state.x =  0;
	state.y =  0;
	state.z =  2;
	state.pitch = 0;
	state.roll = 0;
	state.yaw = 3.14159265359 / 2.0;
    state.u =  0;
    state.v =  0;
    double iz, ez;
    bool first = true;
    double dt = 0.02;
    
    for (int i = 0; i < 1; i++)
    {
	    ////////////////////// CALCULATE VELOCITIES ////////////////////

	    double vel[3] = {sp[_X]-state.x, sp[_Y]-state.y, sp[_Z]-state.z};
	    double rot[3] = {state.roll, state.pitch, state.yaw};
	    n2b(rot,vel);
		double bx = vel[0];
		double by = vel[1];

	    ////////////////////// P ROLL CONTROLLER ////////////////////////

	    double desu = limit(_Kxy*bx,-_maxv,_maxv);
	    double desP = limit(-_Kv*(desu - state.u), -_maxtilt, _maxtilt);
	   
	    ////////////////////// P PITCH CONTROLLER ////////////////////////

	    double desv = limit(_Kxy*by,-_maxv,_maxv);
	    double desR = limit(_Kv*(desv - state.v), -_maxtilt, _maxtilt);
	    
	    //////////////////////// P YAW CONTROLLER ////////////////////////

	    double desY = limit(_Kya * (sp[_YAW] - state.yaw), -_maxyawrate, _maxyawrate);
	    
	    /////////////////// PID THROTTLE CONTROLLER //////////////////////

	    // Get the (P)roportional component (Changed by Andrew)
	    double ez_ = sp[_Z] - state.z;

	    // Get the (I)ntegral component
	    iz = iz + ez_ * dt;
	    
	    // Get the (D)erivative component
	    double de_ = (first ? 0 : (ez_ - ez) / dt);
	    double desth = _th_hover + _Kpz * ez_ + _Kiz * iz + de_ * _Kdz;
	    double th = limit(desth,0,1);
	    
	    // Save (P) contribution for next derivative calculation
	    ez = ez_;

	    // Save (I) contribution for next derivative calculation
	    iz = iz - (desth - th) * 2.0;

	    //////////////////////// CONTROL PACKAGING /////////////////////////

	    // This is no longer the first iteration
	    first = false;

	    // This will be returned
	    std::cout << "[" << i << "] P " << desP << " R " << desR << " T " << th << " Y " << desY << " bx " << bx << " by " << by << std::endl;
	    // ROLL NEEDS NEGATING
	    // PITCH IS FINE
	    // YAW NEEDS NEGATION
	}

	// Success
	return 0;
}
