// State and control messages
#include <hal_quadrotor/control/Controller.h>

using namespace hal::quadrotor;

double Controller::limit(const double& val, const double& minval, const double& maxval)
{
    if (val < minval) return minval;
    if (val > maxval) return maxval;
    return val;
}

// Rotation from navigation to body frame
void Controller::n2b(double rot[3], double vec[3])
{
    double tmp[3];
    for (int i = 0; i < 3; i++)
        tmp[i] = vec[i];
    double sph = sin(rot[0]); double cph = cos(rot[0]);
    double sth = sin(rot[1]); double cth = cos(rot[1]); 
    double sps = sin(rot[2]); double cps = cos(rot[2]);
    double dcm[3][3];
    dcm[0][0] = cth*cps;
    dcm[0][1] = cth*sps;
    dcm[0][2] = -sth;
    dcm[1][0] = sph*sth*cps - cph*sps;
    dcm[1][1] = sph*sth*sps + cph*cps;
    dcm[1][2] = sph*cth;
    dcm[2][0] = cph*sth*cps + sph*sps;
    dcm[2][1] = cph*sth*sps - sph*cps;
    dcm[2][2] = cph*cth;
    for (int i = 0; i < 3; i++)
    {   
        vec[i] = 0;
        for (int j = 0; j < 3; j++)
            vec[i] += dcm[i][j] * tmp[j];
    }
}

// Rotation from body to navigation frame
void Controller::b2n(double rot[3], double vec[3])
{
    double tmp[3];
    for (int i = 0; i < 3; i++)
        tmp[i] = vec[i];
    double sph = sin(rot[0]); double cph = cos(rot[0]);
    double sth = sin(rot[1]); double cth = cos(rot[1]); 
    double sps = sin(rot[2]); double cps = cos(rot[2]);
    double dcm[3][3];
    dcm[0][0] = cth*cps;
    dcm[1][0] = cth*sps;
    dcm[2][0] = -sth;
    dcm[0][1] = sph*sth*cps - cph*sps;
    dcm[1][1] = sph*sth*sps + cph*cps;
    dcm[2][1] = sph*cth;
    dcm[0][2] = cph*sth*cps + sph*sps;
    dcm[1][2] = cph*sth*sps - sph*cps;
    dcm[2][2] = cph*cth;
    for (int i = 0; i < 3; i++)
    {   
        vec[i] = 0;
        for (int j = 0; j < 3; j++)
            vec[i] += dcm[i][j] * tmp[j];
    }
}

// Obtain control from state and timestep
void Controller::GetControl(const hal_quadrotor::State &state, double t, hal_quadrotor::Control &control)
{
    // Save the time 
    double dt = t - tc;

    // If there is too much of a time difference, then its the case
    // that this is (a) the first time the controller has been called,
    // or (b) we have switched from another controller.
    if (fabs(dt) > 1.0)
    {
        // Reset the controller
        Reset();

        // Set the dt to something small / meaningful
        dt = 0;
    }

    // Call the update (sometimes will get a dt == 0)
    Update(state, dt, control);

    // Set tc
    tc = t;
}