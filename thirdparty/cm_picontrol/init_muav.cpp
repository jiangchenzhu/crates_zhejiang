// General ROS functionality
#include <ros/ros.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>


// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

//#include "pi/global.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 
// Quadrotor services
#include <hal_quadrotor/State.h>
#include <hal_quadrotor/SetTruth.h>
#include <hal_quadrotor/GetTruth.h>
#include <hal_quadrotor/GetEstimate.h>


using namespace std;

vector<string> idunits;
int units;
unsigned long int seed = 0;
gsl_rng *r;				// Random seed.
bool arg_seed = false;

// Main entry point of application
int main(int argc, char **argv)
{
	if (argc<2) {
	  ROS_FATAL("requires xml_file as argument"); 
	  return 1;
	}
	else if (argc==2) {
		ROS_INFO("using seed from file");
	}
	else {
		seed = atoi(argv[2]);
		arg_seed = true;
		ROS_INFO("Using seed = %s", argv[2]);
	}
		
	
	try {
		ifstream is(argv[1]);
		// read parameters from xml file
		using boost::property_tree::ptree;
		ptree pt;
		if (!is.is_open()) {
		  ROS_FATAL("xml config file required! %s", argv[1]); 
		  return 1;
		}
		read_xml(is, pt);
		
		// task parameters (only units and idunits are required)
		units = pt.get<int>("units");
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("idunits")) {
			idunits.push_back(v.second.get<string>(""));
		}
		if (!arg_seed) {
			// and seed to start in random position
			seed =pt.get<int>("seed");
		}
	} catch (int e) {
		cout << "Exception" << e << endl;
	}
	
	// Initialise the ROS client
	string nodestr = string("example");
	ros::init(argc, argv, nodestr.c_str()); 
	
	// Create a node handle, which this C code will use to bind to the ROS server
	ros::NodeHandle n;
	
	// Create a client for interacting with the Simulator insert and Resume services
	ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
	ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
	ros::ServiceClient srvPause  = n.serviceClient<sim::Pause> ("/simulator/Pause");

	// insert UVAs
	vector<ros::Subscriber> top_state(units);
	for (int i=0; i<units; i++) {
	
		// Create a resume message, which takes no arguments
		sim::Pause msgPause;
		
		// Call the client with this message
		if (!srvPause.call(msgPause)) {
		  ROS_FATAL("Failed to pause the simulator");
		  return 1;
		}
		
		// Create a message instructing a quadrotor UAV0 to be inserted in to the world
		sim::Insert msgInsert;
		msgInsert.request.model_name = idunits[i];
		msgInsert.request.model_type = "model://hummingbird";
		
		// Call the client with this message
		if (!srvInsert.call(msgInsert))
		{
		  ROS_FATAL("Failed to insert a hummingbird quadrotor into the simulator");
		  return 1;
		}
		
		// You should now see the quadrotor
		cout << idunits[i] << endl;
		ROS_INFO("Successfully inserted the model into the world");

		// Create a resume message, which takes no arguments
		sim::Resume msgResume;

		// Call the client with this message
		if (!srvResume.call(msgResume)) {
			ROS_FATAL("Failed to resume the simulator");
			return 1;
		}
	}

	// init random seed
	gsl_rng_default_seed = seed;
	r = gsl_rng_alloc (gsl_rng_default);

	sleep(2);

	for (int i=0; i<units; i++) {

		// Create a resume message, which takes no arguments
		sim::Pause msgPause;
		
		// Call the client with this message
		if (!srvPause.call(msgPause)) {
		  ROS_FATAL("Failed to pause the simulator");
		  return 1;
		}
	
		hal_quadrotor::SetTruth msgSetTruth;
		string name = string("/hal/quadrotor/hummingbird/") + string(idunits[i]) + string("/SetTruth");
		ros::ServiceClient srvSetTruth = n.serviceClient<hal_quadrotor::SetTruth>(name.c_str());
		
		if (arg_seed) {
			// choose random initial state
			msgSetTruth.request.state.x = gsl_rng_uniform(r)*units - units/2;
			msgSetTruth.request.state.y = gsl_rng_uniform(r)*units - units/2;
		}
		else {
			// choose circular initial state
			double alpha = (double)i*2*3.1415/(double)units;
			msgSetTruth.request.state.x = cos(alpha);
			msgSetTruth.request.state.y = sin(alpha);
		}

		if (i==(units-1)) {
			msgSetTruth.request.state.y = msgSetTruth.request.state.y*5;
			ROS_INFO("mouse coordinates %f,%f", msgSetTruth.request.state.x, msgSetTruth.request.state.y);
		}

	 	if (!srvSetTruth.call(msgSetTruth)) {
			ROS_FATAL("Failed SetTruth");
			return 1;
		}

		// Create a resume message, which takes no arguments
		sim::Resume msgResume;

		// Call the client with this message
		if (!srvResume.call(msgResume)) {
			ROS_FATAL("Failed to resume the simulator");
			return 1;
		}
	
	}

  gsl_rng_free(r);

  // Success!
  return 0;
}
