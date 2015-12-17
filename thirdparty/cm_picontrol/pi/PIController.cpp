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

////////////////////////////////////////////////
// PIController methods
//
double PIController::runningStateReward(
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

vec PIController::predictState(const vec& X, const vec& A)
{
	// Predicts the new state after a control step with A starting in X.
	model->setState(X);
	for (int i=0; i<dtperstep; i++)	model->step(A);
	return model->getState();
}

double PIController::runningControlCost( const vvec& control ) const
{
	// Returns the cost of a control sequence.
	double c=0;
	for (int s=0; s<H; s++)
		c += immediateControlCost(control[s]);
	return c*dS;
}

double PIController::immediateControlCost( const vec& A ) const
{
	// Immediate control cost of the control action A.
	double v=0;
	for(unsigned int i=0; i<A.size(); i++) {
		// v += R[i]*A[i]*A[i];
		v += R*A[i]*A[i];
	}
	return v*.5;
}

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
	stdv_dt = stdv*sqrt(dS/dt);

	dimX = (dimUAVx*units);
	dimU = (dimUAVu*units);
	u_exp.resize(H, vec(dimU,0)); 

	// Initialize control structures
	dimF = dimU+1;
	for (size_t t=0; t<H; t++) {
		A.push_back(MatrixXd(dimU, dimF));
		lhs.push_back(MatrixXd(dimF, dimF));
		dlhs.push_back(MatrixXd(dimF, dimF));
		rhs.push_back(MatrixXd(dimF, dimU));
		drhs.push_back(MatrixXd(dimF, dimU));
	}

	gsl_rng_default_seed = seed;
	r = gsl_rng_alloc (gsl_rng_default);


cout << VERB_LEVEL << "-------------------------------------" << endl;
#ifdef VERB_LEVEL
	char buff[50],buf2[10];
	time_t now = time(NULL);
	strftime(buff, 50, "%Y-%m-%d_%H:%M:%S", localtime(&now));
	sprintf(buf2, "%d_%d_", units, N);
	outfile_costs = "test_results/cost.csv"; //_" + string(buf2) + "_" + string(buff) + ".csv";
	if (VERB_LEVEL >1) {
		outfile_matlab = "test_results/experiment_" + string(buff) + ".m";
		plotSetup();
	}
	// just for debug
	if (VERB_LEVEL <2) {
		ofstream fout(outfile_costs.c_str());//,ios::app);	
		fout << "# " << units << " units," << N << " N," << H << " H," << pt.get<double>("holdingp.minv") << " minv,";
	  fout << pt.get<double>("holdingp.maxv") << " maxv," << pt.get<double>("holdingp.radius");
		fout << " radius," << pt.get<double>("holdingp.hit") << " hit"<< endl;
		fout.close();

		ofstream fout2("files.txt",ios::app);
		fout2 << outfile_costs << endl;
		fout2.close();
	}
#endif

	// Initialize the Model object
	setModel();
	model->setController(this); 
	model->setProperties(); 			
}

vec PIController::computeControlFeedback(const vec &state) {
	// Input is vector of (posx, posy, velx, vely)
	vec action(dimU,0);

	double S_max = -1e100;					// reward of best rollout
	double sum1 = 0.;						// sum of weights
	double sum2 = 0.;						// sum of squared weights
	VectorXd f = VectorXd::Zero(dimF);		// the feedback info
	f(0) = 1.0;								// this allows for an open-loop part
	VectorXd u = VectorXd::Zero(dimU);		// feedback ctl for sampling
	VectorXd noise = VectorXd::Zero(dimU);	// saved noise

	for (int s = 1; s < H; s++) { 			// Move to horizon with current solution
		A[s - 1] = A[s];
	}
	A[H - 1] = MatrixXd::Zero(dimU, dimF);

	for (int n = 0; n < N; n++) {					// make N rollouts
				
		double S = 0;								// reward/value of current rollout
		model->setState(state);						// initialize state of this rollout

		for (int s = 0; s < H; s++) { 				// loop over time with dS step-size

			dlhs[s] = MatrixXd::Zero(dimF, dimF);	// reset
			drhs[s] = MatrixXd::Zero(dimF, dimU);	// reset

			for (int ss = 0; ss < dtperstep; ss++) { // loop over time with dt step-size

				vec X = model->getState();			// state is needed for feedback

				for (int i = 0; i < dimU; i++) {	// generate noise
					noise(i) = gsl_ran_gaussian(r, stdv_dt);
				}

				for (int i = 0; i < units; i++) {	// set feedback information
					// f(0) = 1.0					// allows for a feed-forward component
					f(2*i + 1) = X[4*i + 0];		// x position of unit i
					f(2*i + 2) = X[4*i + 1];		// y position of unit i
				}

				u = A[s]*f;							// feedback-importance control 

				vec perturbed_control(dimU);		// control + noise
				for (int i = 0; i < dimU; i++) {	// type conversion
					perturbed_control[i] = u(i) + noise(i);
				}
				model->step(perturbed_control);	// noisy step
				
				S += model->immediateStateReward()*dt;// increment S with state reward
				S -= dt*R*u.transpose()*(u/2.0 + noise);// decrement S with control cost
				
				drhs[s] += f*(u + noise).transpose(); // save rhs increment
				dlhs[s] += f*f.transpose();			// save covariance increment
			}
			
		} // endl loop over time, rollout is done
		S += model->endStateReward();				// increment S with end reward
		if (S > S_max) {							// rescale the weighted quantities
			double z = exp((S_max - S)/lambda);		// rescale factor
			sum1 *= z;
			sum2 *= z*z;
			for (int s = 0; s < H; s++) {
				lhs[s] *= z;
				rhs[s] *= z;
			}
			S_max = S;								// save max for rescaling and weighting	
		}
		double W;									// weight of the rollout
		if (lambda == 0.0) {						// in deterministic case
			W = 1.0; 
		}
		else {										// regular case
			W = exp((S - S_max)/lambda);			// rescaled weight
		}
		
		// now increment the weighted quantities:
		sum1 += W;
		sum2 += W*W;
		for (int s = 0; s < H; s++) {
			lhs[s] += W*dlhs[s];
			rhs[s] += W*drhs[s];
		}
	}// end loop n = 1...N (rollouts)
	
	// PI CONTROL COMPUTATIONS
cout << "FEEDBACK" << endl;
	for (int i = 0; i < dimU; i++) {				// PI control for current state
		action[i] = rhs[0](0, i)/lhs[0](0, 0);		// optimal action in current state
	}
	double ess = sum1*sum1/sum2;					// effective sample size: 1 <= ess <= N
	for (int s = 1; s < H; s++) {					// computations for feedback matrix A
		double e = 0.0;								// error in the linear solution
		MatrixXd temp;
//		if (ess >= dimU + dimF) {					// sufficient samples for PI feedback computations
//			temp = lhs[s].ldlt().solve(rhs[s]);		// linear solution 
//			e = (lhs[s]*temp - rhs[s]).norm() / rhs[s].norm(); 	// error in the solution
//		}
//		if (e > 1e-12 || ess < dimU + dimF) {		// we don't have a good feedback solution
			A[s] = MatrixXd::Zero(dimU, dimF);
			for (int i = 0; i < dimU; i++) {
				A[s](i, 0) = rhs[s](0, i)/lhs[s](0, 0);// open loop control solution
			}
//		}
//		else {										// we do have a good feedback solution
//			A[s] = temp.transpose();				// set A to the temporary solution
//		}
	}
	#ifdef VERB_LEVEL	
	if (VERB_LEVEL >1) {
		plotCurrent(state,action);
	}
	if (VERB_LEVEL <2) {
		ofstream fout(outfile_costs.c_str(),ios::app);	
		model->setState(state);
		fout << clock() << ",";
		double state_cost = model->immediateStateReward(fout); 
		fout << state_cost << "," << immediateControlCost(action) << endl;
		fout.close();
	}
	#endif
	
	return action;
}



vec PIController::computeControl(const vec &state) {
	// Input is vector of (posx, posy, velx, vely)
	vec action(dimU,0);

	// Move to horizon with exploring controls
	for (int s=1; s<H; s++) { 
		u_exp[s-1] = u_exp[s];
	}
	u_exp[H-1] = vec(dimU,0);

//ofstream fout("log",ios::app);

	// Set value of exploring controls
	double v_exp = runningStateReward(state, u_exp);
//fout << v_exp << endl;
	v_exp -= runningControlCost(u_exp);

//fout << "runningControlCost" << endl;
//fout << v_exp << endl;

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
		v_roll[n] = runningStateReward(state, u_roll);
		v_roll[n] -= runningControlCost(u_roll);

		// improve exploring control if possible
		if ( v_roll[n] > v_exp ) {
			v_exp = v_roll[n];
			u_exp = u_roll;
		}

		// correct value of rollout for to get correct importance sampling.
		v_roll[n] += runningControlCost(noise);

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
//	cout << " end " << endl;
//	printTime();
	
	#ifdef VERB_LEVEL	
	if (VERB_LEVEL >1) {
		plotCurrent(state,action);
	}
	if (VERB_LEVEL <2) {
		ofstream fout(outfile_costs.c_str(),ios::app);	
		model->setState(state);
		fout << clock() << ",";
		double state_cost = model->immediateStateReward(fout); 
		fout << state_cost << "," << immediateControlCost(action) << endl;
		fout.close();
	}
	#endif

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
	ofstream fout(outfile_matlab.c_str(),ios::trunc);
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
	ofstream fout(outfile_matlab.c_str(),ios::app);
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

