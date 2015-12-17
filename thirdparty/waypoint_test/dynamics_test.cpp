#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/ros.h>

// Rotation from body to navigation frame
void n2b(double rot[3], double vec[3])
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
void b2n(double rot[3], double vec[3])
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

// Main entry point
int main(int argc, char* argv[])
{
    // We are in science here :)
    std::cout.precision(16);


    double vec[3] = {1.0,1.0,1.0};
    double rot[3] = {0.0,3.1415/4.0,3.1415/4.0};


    ROS_INFO("ORG: %f %f %f",vec[0],vec[1],vec[2]);
    n2b(rot, vec);
    ROS_INFO("N2B: %f %f %f",vec[0],vec[1],vec[2]);
    b2n(rot, vec);
    ROS_INFO("B2N: %f %f %f",vec[0],vec[1],vec[2]);

    // Success
    return 0;
}

