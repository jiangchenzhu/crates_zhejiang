/*
 * ModelCatMouse.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: vgomez, Sep Thijssen
 */

#ifndef MODEL_CATMOUSE_H_
#define MODEL_CATMOUSE_H_

#include "PIController.h"

using namespace std;

// nested class Model used by the PIController to simulate dynamics
class ModelCatMouse: public PIController::Model {
protected:
	double x;		// coordinate x of the center of the holding pattern
	double y;		// coordinate y of the center of the holding pattern
	double radius;	// radious of the holding pattern
	double minv;	// determines min velocity
	double maxv;	// determines max velocity
	double maxvmouse;	// determines max velocity of the mouse
	double hit;		// penalty constant for collision

	double cvel;
	double cdist;
	double chit;
	double cmouse; 

public:

	// apply control A on the current state
	void step(const vec &A) {

		int units = PIController::units;
		double dt = PIController::dt;

		// mouse: INDEXED AS LAST UNIT
		////////////////////////////////////////
		double dx,dy,DX,DY,d,speed;
		DX=DY=0;
		for(int i=0; i<units-1; i++) {
			dx = state[4*units-4] - state[4*i+0];
			dy = state[4*units-3] - state[4*i+1];
			d = dx*dx+dy*dy;	// distance to cat
			DX += dx/d;			// X away from cat inv proportional to distance
			DY += dy/d;			// Y away from cat inv proportional to distance
		}
		speed = sqrt(DX*DX+DY*DY); // speed of mouse
		// The speed of the mouse is bounded at max_speed.
		if (speed > maxvmouse) {
			DX *= maxvmouse/speed;
			DY *= maxvmouse/speed;
		}
		state[4*units-2] = DX;	// mouse speed X direction
		state[4*units-1] = DY;	// mouse speed Y direction
		state[4*units-4] += state[4*units-2]*dt;	//  X position
		state[4*units-3] += state[4*units-1]*dt;	//  Y position

		// cats
		////////////////////////////////////////
		for(int i=0; i<units-1; i++) {

			state[4*i+0] += state[4*i+2]*dt;	//  X position
			state[4*i+1] += state[4*i+3]*dt;	//  Y position

			state[4*i+2] += A[2*i+0]*dt;		//  X velocity
			state[4*i+3] += A[2*i+1]*dt;		//  Y velocity

		}
	}

	// Output is the immediate state reward. Input is a state.
	double immediateStateReward() {
		cvel = 0; cdist = 0; chit = 0; cmouse = 0; 

		//cout << "in ModelCatMouse::inmediateStateReward()" << endl;
		// Immediate state reward of the state.
		// Returns the negative of the cost c in this case.
		double d,c = 0;
		int units = PIController::units;

		// cost per unit
		for(int i=0; i<units; i++)
		{
			double speed = sqrt(state[4*i+2]*state[4*i+2]+state[4*i+3]*state[4*i+3]);
			if (i != units-1) { // cats
				cvel += exp(speed-maxv); 		// determines max allowed speed
				cvel += exp(-speed+minv);		// determines min allowed speed
			}
//			else {
//				cvel += exp(speed-maxvmouse); 		// determines max allowed speed
//				cvel += exp(-speed+minvmouse);		// determines min allowed speed
//			}

			//// penalty for going too far away
			d = sqrt(state[4*i+0]*state[4*i+0] + state[4*i+1]*state[4*i+1]);
			cdist += exp(d-radius);		// max allowed distance

			// penalty for collision
			for(int j=i+1; j<units; j++) {

				// distance from i -> j
				d = (state[4*i+0]-state[4*j+0])*(state[4*i+0]-state[4*j+0]) +
						(state[4*i+1]-state[4*j+1])*(state[4*i+1]-state[4*j+1]);
				d = sqrt(d);
				if (0.001 > d) d=0.001;
				// penalty for crashing i with j
				chit += hit/d;

				// penalty for going too far away from the mouse
				if (j == units-1) { 	// i.e. j is mouse
					cmouse += d; 			// cost cat i for distance to mouse
				}
			}
		}
		c = cvel + cdist + chit + cmouse;
		return -c;
	}

	double immediateStateReward(std::ofstream &of) {
		double c = immediateStateReward();
		of << cvel << "," << cdist << "," << chit << "," << cmouse << ",";
		return c;
	}


	virtual void setProperties()
	{
		cout << " setting properties" << endl;
		x = ctrl->getProperties().get<double>("catmouse.x");
		y = ctrl->getProperties().get<double>("catmouse.y");
		minv = ctrl->getProperties().get<double>("catmouse.minv");
		maxv = ctrl->getProperties().get<double>("catmouse.maxv");	
		maxvmouse = ctrl->getProperties().get<double>("catmouse.maxvmouse");	
		radius = ctrl->getProperties().get<double>("catmouse.radius");	
		hit = ctrl->getProperties().get<double>("catmouse.hit");	
	}

};

void PIController::setModel()
{
	cout << "setting model for ModelCatMouse" << endl;
	Model *m = new ModelCatMouse();
	model = m;
}

void PIController::unsetModel()
{
	cout << "unsetting model for ModelCatMouse" << endl;
	delete model;
}

#endif /* MODEL_CATMOUSE_H_ */
