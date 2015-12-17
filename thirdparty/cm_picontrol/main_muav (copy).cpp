// General ROS functionality
#include <ros/ros.h>

// PI
#include "pi/PIController.h"
#include "pi/global.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 
#include <boost/regex.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include <algorithm>
// END PI

// Quadrotor services
#include <hal_quadrotor/State.h>

// Quadrotor controllers
#include <hal_quadrotor/Hover.h>
#include <hal_quadrotor/control/Takeoff.h>
#include <hal_quadrotor/control/VelocityHeight.h>

#include <hal_quadrotor/GetEstimate.h>
#include <hal_quadrotor/SetTruth.h>

#define EPS_POS			5e-1	// epsilon position
#define EPS_VEL			1e-3	// epsilon position

//#define DEBUG_INFO 0

typedef enum { IDLE, TAKEOFF, VELHEIGHT, HOVER, WAYPOINT, OTHER } FSM;

using namespace std;

vector<ros::ServiceClient> srvTakeOff;
vector<ros::ServiceClient> srvHover;
vector<ros::ServiceClient> srvVelocityHeight;

vector<FSM> state_FSM; // Finite State Machine state (TODO: get the controller from the msg)  
vector<double> jstate;

// used to safely exit the program
bool stop;

// PI
PIController pi;
// END PI

vector<double> heights; 
vector<string> idunits;
int units;

boost::mutex guard;

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

// Callback for quadrotor state
void StateCallback(int unit, const hal_quadrotor::State::ConstPtr& msg) 
{
	if (msg->controller == (std::string) "Hover" || msg->controller == (std::string) "VelocityHeight")
	{
		state_FSM[unit]  = HOVER;
				// angles of the platform
		double rot[3] = {msg->roll, msg->pitch, msg->yaw};
		
		// put BODY velocity in vel, and rotate to NAVIGATIONAL velocity
		double vel[3] = {msg->u, msg->v, msg->w};
		b2n(rot, vel);

		// update internal state
		jstate[unit*4+0] = msg->x;
		jstate[unit*4+1] = msg->y;
		jstate[unit*4+2] = vel[0];
		jstate[unit*4+3] = vel[1];
#ifdef DEBUG_INFO
		ROS_INFO("UAV %d got state (x, y, dx, dy) = (%f, %f, %f, %f)", unit, msg->x, msg->y, vel[0], vel[1]);
#endif
	}
}

void stopCallback(const std_msgs::String::ConstPtr& msg)
{
#ifdef DEBUG_INFO
	ROS_INFO("Received: PI_STOP.\nSwitching all units to Hover.");
#endif
	stop = true;
}


// Main entry point of application
int main(int argc, char **argv)
{
	if (argc!=2) {
	  ROS_FATAL("requires xml_file as argument"); 
	  return 1;
	}

	try {
		ifstream is(argv[1]);
		// read parameters from xml file
		using boost::property_tree::ptree;
		ptree pt;
		if (!is.is_open()) {
		  ROS_FATAL("xml config not found"); 
		  return 1;
		}
		read_xml(is, pt);

		// get vector of desired heights			
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("takeoffz"))
			heights.push_back(v.second.get<double>(""));

		// get vector of ids 	
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("idunits"))
			idunits.push_back(v.second.get<string>(""));
		
		// initialize Joint State variable
		units = pt.get<int>("units");
		jstate.resize( units*pt.get<int>("dimUAVx") );

		// Initialize PI controller
		pi.Init(pt);

		//double T = pt.get<double>("T");

	} catch (int e) {
		cout << "Exception" << e << endl;
	}
 
	// used to safely exit the program
	stop = false;

	// Initialise the ROS client
	string nodestr = string("main_muav");
	ros::init(argc, argv, nodestr.c_str()); 
	
	// Create a node handle, which this C code will use to bind to the ROS server
	ros::NodeHandle n;
	
	// Try and get a list of topics currently being braodcast
	ros::master::V_TopicInfo topics;
	int units_found = 0;
	vector<string> names;
	vector<ros::Subscriber> top_state;
	
	// Iterate over the topics, checking for entities on /hal/<type>/<model>/<id>/Estimate
	if (ros::master::getTopics(topics)) { 
		for (ros::master::V_TopicInfo::iterator i = topics.begin(); i != topics.end(); i++) {
			boost::cmatch mr;
			if (boost::regex_match(i->name.c_str(), mr, boost::regex("^/hal/(.+)/(.+)/(.+)/Estimate$"))) {
				ROS_INFO("%s -> %s", i->name.c_str(), i->datatype.c_str());
				names.push_back(i->name);
				units_found++;
			}
		}
	}
	
	// check if number of units found corresponds to the expected units (defined in the xml)
	if ( units_found != units ) {
	  ROS_FATAL("Units found %d does not match units defined in the config %d: ", units_found, units);
	  return 1;
	}

	// sort the names (currently required, since the joint state is indexed by unit)
	sort(names.begin(), names.end());
	
	// Subscribe to a stop message
	ros::Subscriber sub_stop = n.subscribe("PI_STOP", 1000, stopCallback);

	// subscribe and add controllers for each unit
	for (int i=0; i<units; i++) {

		// init Finite State Machine
		state_FSM.push_back(IDLE);
		
		ROS_INFO("%s",names[i].c_str());

		// Subscribe to the states of the quadrotors (requires explicit)
		top_state.push_back( n.subscribe<hal_quadrotor::State>(
			names[i].c_str(),10,boost::bind(StateCallback, i, _1))
		);
		ROS_INFO("%s subscribed!", names[i].c_str());

		// Add TakeOff controller		
		string takeoff_cmd = string("/hal/quadrotor/hummingbird/") + idunits[i] + string("/controller/Takeoff");
		srvTakeOff.push_back(n.serviceClient<hal_quadrotor::Takeoff>(takeoff_cmd.c_str()) );
	
		// Add Hover controller		
		string hover_cmd = string("/hal/quadrotor/hummingbird/") + idunits[i] + string("/controller/Hover");
		srvHover.push_back(n.serviceClient<hal_quadrotor::Hover>(hover_cmd.c_str()) );
	
		// Add VelocityHeight controller		
		string velheight_cmd = string("/hal/quadrotor/hummingbird/") + idunits[i] + string("/controller/VelocityHeight");
		srvVelocityHeight.push_back( n.serviceClient<hal_quadrotor::VelocityHeight>(velheight_cmd.c_str()) );
	}		


	// Call Takeoff on each unit
	for (int i=0; i<units; i++) {
		hal_quadrotor::Takeoff msgTakeoff;	
		msgTakeoff.request.altitude = heights[i];
		if(!srvTakeOff[i].call(msgTakeoff)){
			ROS_FATAL("NO TAKE OFF!!");
		} else {
			ROS_INFO("Taking OFF!!");
		}
		state_FSM[i] = TAKEOFF;
	}



	// Keep going indefinitely
	ros::Rate r(15);
    while (ros::ok())
    {
    	// Collect all state estimates
		ros::spinOnce();

		// check stop condittion
		if (stop)
		{
			bool success = true;
			// Call Hover on each unit
			for (int i=0; i<units; i++) 
			{
				hal_quadrotor::Hover msgHover;	
				if(!srvHover[i].call(msgHover))
				{
					success = false;
					ROS_FATAL("NO Hover for UAV %d!!", i);
				} 
			}
			if(success)
			{
				ROS_INFO("Succesfully switched all units to Hover.");
				break;
				return 0;
			}
		}

		// Check that units are ready to receive VelocityHeight commands
		bool ready = true;
		for (int i=0; i<units; i++) {
			ready = ready && (state_FSM[i] == HOVER);
		}
		if (ready)
		{
			// Path Integral controller (based on VelocityHeight)
			vec X = jstate;
			//vec action = pi.computeControl(X);
			vec action = pi.computeControlFeedback(X);
			vec Xp = pi.predictState(X, action);
		
			// Distribute commands
			for (int i=0; i<units; i++) {
			
				// convert pi control to VelocityHeight control
				hal_quadrotor::VelocityHeight msgVelocityHeight;
				msgVelocityHeight.request.dx  = Xp[i*4 + 2];
				msgVelocityHeight.request.dy  = Xp[i*4 + 3];
				msgVelocityHeight.request.z   = heights[i];
				msgVelocityHeight.request.yaw = 0.;
			
#ifdef DEBUG_INFO
				ROS_INFO("UAV %d sent command (dx, dy, z, yaw) = (%f, %f, %f, %f)", 
					i, msgVelocityHeight.request.dx, msgVelocityHeight.request.dy, 
					msgVelocityHeight.request.z, msgVelocityHeight.request.yaw);
#endif

				if(!srvVelocityHeight[i].call(msgVelocityHeight)){
					ROS_FATAL("NO VELHEIGHT!!");
				}	
			}
		}

		// Sleep
		r.sleep();
    }
	
	ROS_INFO("Terminating PI control (muav_main).");
	ros::spin();
	
	// Success!
	return 0;
}
