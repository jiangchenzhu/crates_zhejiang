/*
 * PIController.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: vgomez, Sep Thijssen
 */

#include "PIController.h"

//#define OLD_CONTROL 1

using namespace std;

// static members initialization
double PIController::dt = .0;
int PIController::units = 0;
double PIController::R = .0;
double PIController::nu = .0;
double PIController::lambda = .0;
int PIController::dtperstep = 0;
int PIController::H = 0;
double PIController::dS = 0.;
double PIController::stdv = 0.;
int PIController::dimUAVx = 0.;
int PIController::dimUAVu = 0.;
int PIController::seed = 0;

////////////////////////////////////////////////
// Model methods
//	step() and immediateStateReward() to be defined in the concrete Model class
//

double PIController::Model::immediateControlCost( const vec& A ) const
{
	// Immediate control cost of the control action A.
	double v=0;
	for(unsigned int i=0; i<A.size(); i++) {
		// v += R[i]*A[i]*A[i];
		v += R*A[i]*A[i];
	}
	return v*.5;
}

////////////////////////////////////////////////
// Sampler methods
//

double PIController::Sampler::runningStateReward(
		const vec& X0,
		const vvec& control
) {
	// Returns state based reward of a rollout.
	model->setState(X0);
	double v=0;
	for (int s=0; s<H; s++) {
		for (int i=0; i<dtperstep; i++){
			v += model->immediateStateReward()*dt;
			model->step( control[s] );
		}
	}
	v += model->endStateReward();
	return v;
}

vec PIController::Sampler::predictState(const vec& X, const vec& A)
{
	// Predicts the new state after a control step with A starting in X.
	model->setState(X);
	for (int i=0; i<dtperstep; i++)	model->step(A);
	return model->getState();
}

double PIController::Sampler::runningControlCost( const vvec& control ) const
{
	// Returns the cost of a control sequence.
	double c=0;
	for (int s=0; s<H; s++)
		c += model->immediateControlCost(control[s]);
	return c*dS;
}

////////////////////////////////////////////////
// PIController methods
//

void PIController::Init(const boost::property_tree::ptree &_pt)
{
	pt = _pt;

	dt =pt.get<double>("dt");
	R =pt.get<double>("R");
	dtperstep =pt.get<int>("dtperstep");
	H =pt.get<int>("H");
	nu =pt.get<double>("nu");
	units = pt.get<int>("units");
	dimUAVx =pt.get<int>("dimUAVx");
	dimUAVu =pt.get<int>("dimUAVu");
	seed =pt.get<int>("seed");
	N = pt.get<int>("N");

	lambda = R * nu;
	dS = dt * dtperstep;
	stdv = sqrt(nu/dS);

	dimX = (dimUAVx*units);
	dimU = (dimUAVu*units);
	u_exp.resize(H, vec(dimU,0)); 

	gsl_rng_default_seed = seed;
	r = gsl_rng_alloc (gsl_rng_default);
	time_t timer;
	time(&timer);
	char name[50];
	sprintf(name,"experiment%d.m",(int)timer);
	outfile = name;
	plotSetup();

	// Initialize the sampler object
	sampl.Init(this);
}

PIController::~PIController() {
	// TODO Auto-generated destructor stub
}

vec PIController::computeControl(const vec &state) {
	// Input is vector of (posx, posy, velx, vely)
	vec action(dimU,0);

	// Move to horizon with exploring controls
	for (int s=1; s<H; s++) { 
		u_exp[s-1] = u_exp[s];
	}
	u_exp[H-1] = vec(dimU,0);

ofstream fout("log",ios::app);

	// Set value of exploring controls
	double v_exp = sampl.runningStateReward(state, u_exp);
fout << v_exp << endl;
	v_exp -= sampl.runningControlCost(u_exp);

fout << "sampl.runningControlCost" << endl;
fout << v_exp << endl;

	// define some algorithm variables:
	double v_max = -1e100;
	vec  v_roll(N);
	vvec u_roll(H, vec(dimU));
	vvec noise(H, vec(dimU));
	vvec u_init(N, vec(dimU));

	for (int n=0; n<N; n++) {

		// set exploring noise and perturb control.
		for (int s=0; s<H; s++) {
			for(int i=0; i<dimU; i++) {
				noise[s][i] = gsl_ran_gaussian(r,stdv);
				u_roll[s][i] = u_exp[s][i] + noise[s][i];
			}
		}
		// save initial direction
		u_init[n] = u_roll[0];

		// set value of random control
		v_roll[n] = sampl.runningStateReward(state, u_roll);
		v_roll[n] -= sampl.runningControlCost(u_roll);

		// improve exploring control if possible
		if ( v_roll[n] > v_exp ) {
			v_exp = v_roll[n];
			u_exp = u_roll;
		}

		// correct value of rollout for to get correct importance sampling.
		v_roll[n] += sampl.runningControlCost(noise);

		// save max for rescaling weights
		if (v_roll[n] > v_max) v_max = v_roll[n];
	}
	
	// PI update
	double sum1 = 0;	// sum of weights
	double sum2 = 0;	// sum of square weights
	for (int n=0; n<N; n++) {
		double W = v_roll[n] - v_max;
		if (W >= -20*lambda) {
			if (lambda == 0.) W = 1;
			else W = exp(W/lambda);
			sum1 += W;
			sum2 += W*W;
			for (int i=0; i<dimU; i++)
				action[i] += W*u_init[n][i];
		}
		else W = 0;
	}

	// normalization
	for (int i=0; i<dimU; i++)
		action[i] /= sum1;
	cout << " end " << endl;
	printTime();
	plotCurrent(state,action);

	return action;
}


void PIController::printTime() const       // GENERAL
{
	clock_t t=clock();
	clock_t s=t/CLOCKS_PER_SEC;
	t=t%CLOCKS_PER_SEC;
	t=t*100;
	t=t/CLOCKS_PER_SEC;
	cout << "time: "<<s<<".";
	if (t<10) cout<<0;
	cout<<t<<endl;
}


void PIController::plotSetup() const 	// GENERAL
// For export and plotting in matlab.
{	
	// id number of experiment
	ofstream fout(outfile.c_str(),ios::trunc);
	fout << "%%matlab" << endl;
	fout << "%%This file is generated by pi_qrsim.cpp" << endl << endl;
	fout << "%%Moving Horizon stochastic PI control." << endl << endl;
	fout << "%%PARAMETERS:" << endl << endl;
	fout << "%%Horizon steps,\n H=" << H << ";"<< endl;
	fout << "%%Plant Precision" << endl << " dtperstep=" << dtperstep << ";"<< endl;
	fout << "%%Infinitessimal time, " << endl << "dt=" << dt << ";"<< endl;
	fout << "%%Time of a step, " << endl << "dS=dtperstep*dt;" << endl;
	fout << "%%Samples, " << endl << "N=" << N << ";"<< endl;
	fout << "%%DATA:" << endl;
	fout << "X = [];\n U = [];" << endl;
	fout.close();
}

void PIController::plotCurrent(const vec& state, const vec& action) const	
// For export and plotting in matlab
{
	// id number of experiment
	ofstream fout(outfile.c_str(),ios::app);
	fout << "X = [X; [";
	for(int i=0; i<dimX; i++)
		fout << state[i] << " ";
	fout << "]];" << endl;
	fout << "U = [U; [" << endl;
	for(int i=0; i<dimU; i++)
		fout << action[i] << " ";
	fout << "]];" << endl;
	fout.close();
}

