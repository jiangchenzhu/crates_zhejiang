/*
 * global.h
 *
 *  Created on: Apr 5, 2013
 *      Authors: vgomez, Sep Thijssen
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <vector>
#include <math.h>
#include <iostream>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <ctime>

#include <hal_quadrotor/State.h>

#define SIZE_MSG	5

using namespace std;

// Message structure
//	IIIII.XXX.xxx.YYY.yyy.ZZZ.zzz.UUU.uuu.VVV.vvv.WWW.vvv
//	IIIII		identifier of the UVA
//	XXX.xxx	position x
//	YYY.yyy	position y
//	ZZZ.zzz	position z
//	UUU.uuu	velocity x
//	VVV.vvv	velocity y
//	WWW.www	velocity z

typedef vector<double> vec;
typedef vector<vec> vvec;

template<typename G>
ostream& operator<<(ostream& os, const vector<G>& v)
{
	typename vector<G>::const_iterator it;
	for (it=v.begin(); it!=v.end(); it++)
		os << *it << " ";
	os << endl;
	return os;
}

string getState(const hal_quadrotor::State::ConstPtr& msg);

#endif /* GLOBAL_H_ */
