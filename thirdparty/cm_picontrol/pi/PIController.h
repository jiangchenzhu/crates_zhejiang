/*
 * PIController.h
 *
 *  Created on: Mar 31, 2013
 *      Author: vgomez, Sep Thijssen
 */

#ifndef PICONTROLLER_H_
#define PICONTROLLER_H_

#define VERB_LEVEL 1 

#include "global.h"
#include <boost/property_tree/ptree.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <Eigen/Dense> 
using namespace Eigen; 


class PIController {
public:
	static double dt;		// time increment (required in Model)
	static int units;		// number of UAVs (required in PIController and Model)
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
	gsl_rng *r;					// Random seed.
	int dimX;					// dimension state
	int dimU;					// dimension control
	int N;						// number of rollouts applied each step
	double stdv_dt;				// new time scale for noise
	vvec u_exp;					// [H][dimU] current best, used for exploring
	std::string outfile_matlab;	// name of the output .m file
	std::string outfile_costs;	// name of a log file with costs (comma separated values format)
	boost::property_tree::ptree pt;

	// needed for feedback
	vector<MatrixXd> A,lhs,dlhs,rhs,drhs;
	int dimF;

public :

	// nested class Model used by the PIController to simulate dynamics
	class Model {
	protected:
		vec state;			// [dimX] running state
		PIController *ctrl; // pointer to PIController object (outer class)

	public:

		Model() {};
		virtual ~Model() {};

		// apply control uuu on the current state
		virtual void step(const vec&) = 0;

		// replaces the current state
		void setState(const vec& X0) { state = X0; }

		// returns the current state
		vec getState() const { return state; }

		// replaces the current state
		void setController(PIController *_ctrl) { ctrl = _ctrl; }

		// sets the concrete model (implemented by subclass)
		virtual void setProperties() = 0;

		// Output is the immediate state reward
		// writes in of the components of the reward to be logged
		virtual double immediateStateReward(std::ofstream &of) = 0;
		
		// Output is the immediate state reward
		// sets different attributes of the subclass to be logged if necessary 
		virtual double immediateStateReward() = 0;

		// Output is the end state reward. Input is a state.
		double endStateReward() { return immediateStateReward(); };

	};

private:

protected:
	Model *model;			// The dynamical model the PIController uses

	public:
	PIController() {};
	virtual ~PIController() { unsetModel(); };

	void Init(const boost::property_tree::ptree &pt);

	void setSeed(const double &s) { seed = s; };

	// sets the concrete model (implemented by subclass)
	void setModel();

	// unsets the concrete model (implemented by subclass)
	void unsetModel();

	// Returns state based reward of a rollout
	double runningStateReward(const vec&, const vvec&);

	// Returns the cost of a control sequence.
	double runningControlCost(const vvec&) const;

	vec predictState(const vec& X, const vec& A);

	// Output is the immediate control cost. Input is a control action.
	double immediateControlCost(const vec&) const;

//	double immediateStateReward(const vec& X)	{	model->setState(X);	return model->immediateStateReward(); }

	void printTime() const;
	
	void plotCurrent(const vec&, const vec&) const;

	void plotSetup() const;

	boost::property_tree::ptree getProperties() { return pt; }

	// The PI-control algorithm, input is a state, output is control (VelHeight)
	vec computeControl(const vec &);

	// PI-Controller with feedback, input is a state, output is control (VelHeight)
	// property A is updated
	vec computeControlFeedback(const vec &);

};

#endif /* PICONTROLLER_H_ */
