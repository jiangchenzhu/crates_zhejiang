// Standard includes
#include <fstream>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>
#define PI 3.14159265359

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

// T1: Face east, move east
void t1()
{
	double rot[3] = {0, 0, 0};						// Face east
	double vec[3] = {1, 0, 0};						// Move east
	n2b(rot,vec);
	printf("T1: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==1 && round(vec[1])==0 && round(vec[2])==0);	// Expected [1 0 0]
}

// T2: Face north, move north
void t2()
{
	double rot[3] = {0, 0, PI/2};					// Face north
	double vec[3] = {0, 1, 0};						// Move north
	n2b(rot,vec);
	printf("T2: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==1 && round(vec[1])==0 && round(vec[2])==0);	// Expected [1 0 0]
}

// T3: Face north, move east
void t3()
{
	double rot[3] = {0, 0, PI/2};					// Face north
	double vec[3] = {1, 0, 0};						// Move east
	n2b(rot,vec);
	printf("T3: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==0 && round(vec[1])==-1 && round(vec[2])==0);	// Expected [0 -1 0]
}

// T4: Face west, move south
void t4()
{
	double rot[3] = {0, 0, PI};						// Face north
	double vec[3] = {0,-1, 0};						// Move east
	n2b(rot,vec);
	printf("T4: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==0 && round(vec[1])==1 && round(vec[2])==0);	// Expected [0 1 0]
}

// T4: Face south, move north
void t5()
{
	double rot[3] = {0, 0, -PI/2};					// Face south
	double vec[3] = {0, 1, 0};						// Move north
	n2b(rot,vec);
	printf("T5: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==-1 && round(vec[1])==0 && round(vec[2])==0);	// Expected [-1 0 0]
}

// T4: Face west, move west
void t6()
{
	double rot[3] = {0, 0, -PI};					// Face south
	double vec[3] = {-1, 0, 0};						// Move north
	n2b(rot,vec);
	printf("T6: %f %f %f \n",vec[0],vec[1],vec[2]);
	assert(round(vec[0])==1 && round(vec[1])==0 && round(vec[2])==0);	// Expected [1 0 0]
}

// Main entry point
int main(int argc, char* argv[])
{	
	// Facing East
	t1();
	t2();
	t3();
	t4();
	t5();
	t6();

	return 1;
}
