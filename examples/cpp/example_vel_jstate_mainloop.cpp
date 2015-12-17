// General ROS functionality
#include <ros/ros.h>

// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

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
//#include <hal_quadrotor/control/Waypoint.h>
//#include <hal_quadrotor/control/Hover.h>

#define EPS_POS			5e-1	// epsilon position
#define EPS_VEL			1e-3	// epsilon position

typedef enum { IDLE, TAKEOFF, VELHEIGHT, HOVER, WAYPOINT, OTHER } FSM;

vector<ros::ServiceClient> srvTakeOff;
vector<ros::ServiceClient> srvVelocityHeight;

using namespace std;

vector<FSM> state_FSM; // Finite State Machine state (TODO: get the controller from the msg)  
vector<double> jstate;

// PI
PIController pi;
// END PI

vector<double> heights; 
vector<string> idunits;
int units;

// compares two states 
bool sameState(const hal_quadrotor::State::ConstPtr& s1, const hal_quadrotor::State &s2)
{
	double dpos = sqrt((s1->x-s2.x)*(s1->x-s2.x) + (s1->y-s2.y)*(s1->y-s2.y) + (s1->z-s2.z)*(s1->z-s2.z));
	double dvel = sqrt((s1->u-s2.u)*(s1->u-s2.u) + (s1->v-s2.v)*(s1->v-s2.v));
 	ROS_INFO("dpos: [%f %f]", dpos, dvel);
	return ((dpos < EPS_POS) && (dvel < EPS_VEL));
}

bool sameHeight(const hal_quadrotor::State::ConstPtr& s1, const hal_quadrotor::State &s2)
{
	double dpos = sqrt((s1->z-s2.z)*(s1->z-s2.z));
	double dvel = sqrt((s1->u-s2.u)*(s1->u-s2.u) + (s1->v-s2.v)*(s1->v-s2.v));
	return ((dpos < EPS_POS) && (dvel < EPS_VEL));
}

// Callback for quadrotor state
void StateCallback(int unit, const hal_quadrotor::State::ConstPtr& msg) 
{
	switch (state_FSM[unit]) {
		case IDLE: { 
  		ROS_INFO("IDLE, trying to take off");
			hal_quadrotor::Takeoff msgTakeoff;	
			msgTakeoff.request.altitude = heights[unit];
			if(!srvTakeOff[unit].call(msgTakeoff)){
				ROS_FATAL("NO TAKE OFF!!");
			} else {
				ROS_INFO("Taking OFF!!");
				state_FSM[unit] = TAKEOFF;
			}
		}	break;
		case TAKEOFF: {
			hal_quadrotor::State s;
			s.x = 5.; s.y = 5.; s.z = heights[unit];
			s.u = 0.; s.v = 0.;
			if (sameHeight(msg, s)) {
				ROS_INFO("TAKEN OFF, switch to HOVER");
				state_FSM[unit] = HOVER;
			}
		} break;
		case HOVER: {
			// in HOVER we will actually do PI control
  		ROS_INFO("HOVER pos: [%f,%f,%f]", msg->x, msg->y, msg->z);
			jstate[unit*4] = msg->x;
			jstate[unit*4+1] = msg->y;
			jstate[unit*4+2] = msg->u;
			jstate[unit*4+3] = msg->v;
		} break;
		default: {
  		ROS_INFO("pos: [%f,%f,%f]", msg->x, msg->y, msg->z);
		} break;
	}
	
}

// Main entry point of application
int main(int argc, char **argv)
{
	// PI
	if (argc!=2) {
		cout << "requires xml_file as argument" << endl;
		exit(-1);
	}

	try {
		ifstream is(argv[1]);
		// read parameters from xml file
		using boost::property_tree::ptree;
		ptree pt;
		if (!is.is_open()) {
			cout << "file not found" << endl;
			exit(-1);
		}
		read_xml(is, pt);
		
//	// task parameters
//	double dt = pt.get<double>("dt");
//	double R = pt.get<double>("R");
//	int dtperstep = pt.get<int>("dtperstep");
//	int H = pt.get<int>("H");
//	double nu = pt.get<double>("nu");
//	double T = pt.get<double>("T");
//	int seed = pt.get<int>("seed");
//	int N = pt.get<int>("N");
//	units = pt.get<int>("units");
//	int dimUAVx = pt.get<int>("dimUAVx");
//	int dimUAVu = pt.get<int>("dimUAVu");
//	BOOST_FOREACH(ptree::value_type &v,	pt.get_child("takeoffz")) {
//		heights.push_back(v.second.get<double>(""));
//	}
//	BOOST_FOREACH(ptree::value_type &v,	pt.get_child("idunits")) {
//		idunits.push_back(v.second.get<string>(""));
//	}

//	jstate.resize( units*dimUAVx );
//
//	double dt_qrsim = 1.; // CHANGE
//	double dS = dt*dtperstep;
//	int dtperstep_qrsim = int(dS/dt_qrsim);
//	double dS_qrsim = dt_qrsim*dtperstep_qrsim;

// 	// initialize static variables
//	PIController::dt = dt;			
//	PIController::units = units;		
//	PIController::R = R;
//	PIController::nu = nu;
//	PIController::lambda = R*nu;
//	PIController::dtperstep = dtperstep;
//	PIController::H = H;
//	PIController::dS = dt*dtperstep;
//	PIController::stdv = sqrt(nu/dS);
	
		// Initial state X should be input as well (don't forget tpo convert)
		// create instance of PIController
		//pi.Init(dimUAVx, dimUAVu,	seed,	N);
		// Initial state X should be input as well (don't forget tpo convert)
		// create instance of PIController
		pi.Init(pt);//prop);

		double T = pt.get<double>("T");
		
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
	vector<ros::Subscriber> top_state;
	for (int i=0; i<units; i++) {
		// Subscribe to the states of the quadrotors (requires explicit)
		string event = string("hal/") + idunits[i] + string("/Estimate");
		top_state.push_back( n.subscribe<hal_quadrotor::State>(
				event,
				1000,
				boost::bind(StateCallback, i, _1)
			)
		);

		// init Finite State Machine
		state_FSM.push_back(IDLE);
	
		// Create a resume message, which takes no arguments
		sim::Pause msgPause;
		
		// Call the client with this message
		if (!srvPause.call(msgPause))
		{
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
	  if (!srvResume.call(msgResume))
	  {
	    ROS_FATAL("Failed to resume the simulator");
	    return 1;
	  }
	}

  // rosservice call /hal/UAV0/controller/Takeoff 5
	for (int i=0; i<units; i++) {
		string takeoff_cmd = string("hal/") + idunits[i] + string("/controller/Takeoff");
		srvTakeOff.push_back( n.serviceClient<hal_quadrotor::Takeoff>(takeoff_cmd.c_str()) );
	
		string velheight_cmd = string("hal/") + idunits[i] + string("/controller/VelocityHeight");
		srvVelocityHeight.push_back( n.serviceClient<hal_quadrotor::VelocityHeight>(velheight_cmd.c_str()) );
	}
	
  //ros::Rate loop_rate(1);

  while (ros::ok())
  {
  	// Instead of sleep we use our PI controller
    //loop_rate.sleep();

		// During sleep (or PI-control) all messages get buffered. A spin will cause the Callbacks to work through the buffered messages. 
    ros::spinOnce();

		// pi controller (based on VelocityHeight)
		// convert state to pi state

		bool ready = true;
		for (int i=0; i<units; i++) {
			ready = ready && (state_FSM[i] == HOVER);
		}

		if (ready) {
			vec X = jstate;
			vec action = pi.computeControl(X);
			vec Xp = pi.predictState(X, action);
	
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

  }

  // Success!
  return 0;
}
