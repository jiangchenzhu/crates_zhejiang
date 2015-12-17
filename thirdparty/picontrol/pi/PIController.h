/*
 * PIController.h
 *
 *  Created on: Mar 31, 2013
 *      Author: vgomez, Sep Thijssen
 */

#ifndef PICONTROLLER_H_
#define PICONTROLLER_H_

#include "global.h"
#include <boost/property_tree/ptree.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

class PIController {
public:
	static double dt;		// time increment (required in Model)
	static int units;		// number of UAVs (required in Sampler and Model)
	static double R;	    // control (required in Model)
	static double nu;		// noise level
	static double lambda;  	// PI scalar ratio = nu*R
	static int dtperstep;	// number of dt per control step
	static int H;			// number of steps in horizon
	static double dS;		// duration of a control step = dt*dtperstep
	static double stdv;		// local standard deviation = sqrt(nu/dS)
	static int dimUAVx;
	static int dimUAVu;
	static int seed;

protected:
	gsl_rng *r;				// Random seed.
	int dimX;				// dimension state
	int dimU;				// dimension control
	int N;					// number of rollouts applied each step
	vvec u_exp;				// [H][dimU] current best, used for exploring
	std::string outfile;	// name of the output .m file
	boost::property_tree::ptree pt;

public :

	// nested class Model used by the sampler to simulate dynamics
	class Model {
	protected:
		vec state;			// [dimX] running state
		PIController *ctrl; // pointer to PIController object (outer class)

	public:

		Model() {};
		virtual ~Model() {};

		// apply control uuu on the current state
		virtual void step(const vec&) = 0;//, const double&);

		// replaces the current state
		void setState(const vec& X0) { state = X0; }

		// returns the current state
		vec getState() const { return state; }

		// replaces the current state
		void setController(PIController *_ctrl) { ctrl = _ctrl; }

		// sets the concrete model (implemented by subclass)
		virtual void setProperties() = 0;

		// Output is the immediate control cost. Input is a control action.
		double immediateControlCost(const vec&) const;

		// Output is the immediate state reward. Input is a state.
		virtual double immediateStateReward() const = 0; //const vec&) const;

		// Output is the end state reward. Input is a state.
		double endStateReward() const { return immediateStateReward(); };//const vec &X) const { return immediateStateReward(X); };

	};

private:

	// nested class Sampler used by PIController to sample trajectories
	class Sampler {
	protected:
		Model *model;			// The dynamical model the sampler uses

	public:
		
		Sampler() {};
		virtual ~Sampler() { unsetModel(); };

		void Init(PIController *ctrl) {
			setModel();
			model->setController(ctrl); 
			model->setProperties(); 			
		}
//		vec getState() const { return model.getState(); }

		// sets the concrete model (implemented by subclass)
		void setModel();

		// unsets the concrete model (implemented by subclass)
		void unsetModel();

		// Returns state based reward of a rollout
		double runningStateReward(const vec&, const vvec&);

		// Returns the cost of a control sequence.
		double runningControlCost(const vvec&) const;

		vec predictState(const vec& X, const vec& A);
	};

public:
	Sampler sampl;

	PIController() {};
	virtual ~PIController();

	void Init(	const boost::property_tree::ptree &pt );

	void printTime() const;
	
	void plotCurrent(const vec&, const vec&) const;

	void plotSetup() const;

	boost::property_tree::ptree getProperties() { return pt; }

	// The PI-control algorithm, input is a state, output is control (VelHeight)
	vec computeControl(const vec &);

	vec predictState(const vec& X, const vec& A) { return sampl.predictState(X, A); };

};

#endif /* PICONTROLLER_H_ */
