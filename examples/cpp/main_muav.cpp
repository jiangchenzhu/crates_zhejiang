// General ROS functionality
#include <ros/ros.h>

// PI
#include "pi/PIController.h"
#include "pi/global.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 
// END PI

// Quadrotor services
#include <hal_quadrotor/State.h>

// Quadrotor controllers
#include <hal_quadrotor/control/Takeoff.h>
#include <hal_quadrotor/control/VelocityHeight.h>

#include <algorithm>

#define EPS_POS			5e-1	// epsilon position
#define EPS_VEL			1e-3	// epsilon position

typedef enum { IDLE, TAKEOFF, VELHEIGHT, HOVER, WAYPOINT, OTHER } FSM;

using namespace std;

vector<ros::ServiceClient> srvTakeOff;
vector<ros::ServiceClient> srvVelocityHeight;

vector<FSM> state_FSM; // Finite State Machine state (TODO: get the controller from the msg)  
vector<double> jstate;

// PI
PIController pi;
// END PI

vector<double> heights; 
vector<string> idunits;
int units;


// Callback for quadrotor state
void StateCallback(int unit, const hal_quadrotor::State::ConstPtr& msg) 
{
	if (msg->controller == (std::string) "Hover")
	{
		state_FSM[unit]  = HOVER;
		jstate[unit*4]   = msg->x;
		jstate[unit*4+1] = msg->y;
		jstate[unit*4+2] = msg->u;
		jstate[unit*4+3] = msg->v;
	}
}

// This is called at a fixed rate of 50Hz
void PILoop(const ros::TimerEvent& event)
{	
	// Check that units are ready to receive VelocityHeight commands
	bool ready = true;
	for (int i=0; i<units; i++) {
		ready = ready && (state_FSM[i] == HOVER);
	}
	if (ready) {
	
		// Path Integral controller (based on VelocityHeight)
		vec X = jstate;
		vec action = pi.computeControl(X);
		vec Xp = pi.predictState(X, action);
	
		// Distribute commands
		for (int i=0; i<units; i++) {
		
			// convert pi control to VelocityHeight control
			hal_quadrotor::VelocityHeight msgVelocityHeight;
			msgVelocityHeight.request.dx = Xp[i*4 + 2];
			msgVelocityHeight.request.dy = Xp[i*4 + 3];
			msgVelocityHeight.request.z = heights[i];
			msgVelocityHeight.request.yaw = 0.;
		
			if(!srvVelocityHeight[i].call(msgVelocityHeight)){
				ROS_FATAL("NO VELHEIGHT!!");
			}	
		}
	}

	if (event.profile.last_duration.toSec() > 0.1)
		ROS_WARN("PI loop duration should be < 0.1s, found to be %f",event.profile.last_duration.toSec());
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

		double T = pt.get<double>("T");

	} catch (int e) {
		cout << "Exception" << e << endl;
	}
 
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
	
	// Iterate over the topics
	if (ros::master::getTopics(topics)) { 
		for (ros::master::V_TopicInfo::iterator i = topics.begin(); i != topics.end(); i++) {

			// Check if type corresponds to a hal_quadrotor/State
			//if (!strcmp("hal_quadrotor/State", i->datatype.c_str())) {
			if (i->name.find("Estimate")!=string::npos) {
				
				// Unit found
				ROS_INFO("%s -> %s", i->name.c_str(), i->datatype.c_str());
				names.push_back(i->name);
				units_found++;
		
				// Extract id unit and sanity check of id
				int pos1 = i->name.find("/hal/")+5;
				int pos2 = i->name.find("Estimate")-1;
				string id = i->name.substr(pos1,pos2-pos1);
				if (find(idunits.begin(), idunits.end(), id.c_str()) == idunits.end()) {
					ROS_FATAL("id unit %s not specified in xml", id.c_str());
				  return 1;
				}
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
	
	// subscribe and add controllers for each unit
	for (int i=0; i<units; i++) {

		// init Finite State Machine
		state_FSM.push_back(IDLE);
		
		// Subscribe to the states of the quadrotors (requires explicit)
		top_state.push_back( n.subscribe<hal_quadrotor::State>(
				names[i].c_str(),
				1000,
				boost::bind(StateCallback, i, _1)
				)
		);
		ROS_INFO("%s subscribed!", names[i].c_str());

		// Add TakeOff controller		
		string takeoff_cmd = string("hal/") + idunits[i] + string("/controller/Takeoff");
		srvTakeOff.push_back(n.serviceClient<hal_quadrotor::Takeoff>(takeoff_cmd.c_str()) );
	
		// Add VelocityHeight controller		
		string velheight_cmd = string("hal/") + idunits[i] + string("/controller/VelocityHeight");
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

    // Immediately start control loop
    ros::Timer timer = n.createTimer(
        ros::Duration(0.1), 
        PILoop
    );

    // Wait here
    ros::spin();
	
	// Success!
	return 0;
}
