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

// TODO: better way to get parameters in the callback instead of global vars
ros::ServiceClient srvTakeOff; 
ros::ServiceClient srvVelocityHeight; 
FSM state_FSM = IDLE;

// PI
using namespace std;
PIController pi;
// END PI

double takeoffx; 
double takeoffy; 			
double takeoffz; 
string idunit;


// compares two states 
bool sameState(const hal_quadrotor::State::ConstPtr& s1, const hal_quadrotor::State &s2)
{
	double dpos = sqrt((s1->x-s2.x)*(s1->x-s2.x) + (s1->y-s2.y)*(s1->y-s2.y) + (s1->z-s2.z)*(s1->z-s2.z));
	double dvel = sqrt((s1->u-s2.u)*(s1->u-s2.u) + (s1->v-s2.v)*(s1->v-s2.v));
 	ROS_INFO("dpos: [%f %f]", dpos, dvel);
	return ((dpos < EPS_POS) && (dvel < EPS_VEL));
}

// Callback for quadrotor state
void StateCallback(const hal_quadrotor::State::ConstPtr& msg)
{
	switch (state_FSM) {
		case IDLE: { 
  		ROS_INFO("IDLE, trying to take off");
			hal_quadrotor::Takeoff req_Takeoff;	
			req_Takeoff.request.altitude = takeoffz;
			if(!srvTakeOff.call(req_Takeoff)){
				ROS_FATAL("NO TAKE OFF!!");
			} else {
				ROS_INFO("Taking OFF!!");
				state_FSM = TAKEOFF;
			}
		}	break;
		case TAKEOFF: {
			hal_quadrotor::State s;
			s.x = 5.; s.y = 5.; s.z = takeoffz;
			s.v = 0.; s.w = 0.;
			if (sameState(msg, s)) {
				ROS_INFO("TAKEN OFF, switch to VelocityHeight");
				state_FSM = VELHEIGHT;
			}
		} break;
		case VELHEIGHT: {
  		ROS_INFO("pos: [%f,%f,%f]", msg->x, msg->y, msg->z);
		  // rosservice call /hal/UAV0/controller/VelocityHeight
			hal_quadrotor::VelocityHeight req_VelocityHeight;
			req_VelocityHeight.request.dx = .0;
			req_VelocityHeight.request.dy = .1;
			req_VelocityHeight.request.z = takeoffz;
			req_VelocityHeight.request.yaw = 0.;
			if(!srvVelocityHeight.call(req_VelocityHeight)){
				ROS_FATAL("NO VELHEIGHT!!");
			} else {
				ROS_INFO("VelocityHeight command sent!!");
				state_FSM = OTHER;
			}
		} break;
		case OTHER: {
			// PI
			// pi controller (based on VelocityHeight)
			// convert state to pi state
			// (pi state should be the most updated joint state)
			vec s(4);
			s[0] = msg->x; s[1] = msg->y;
			s[2] = msg->u; s[3] = msg->v;
cout << "Sending state to PI controller!!" << endl;
			vec action = pi.computeControl(s);
cout << "current state " << s << endl;
cout << action << endl;
			vec sp = pi.predictState(s, action);
cout << "next state " << sp << endl;
			
			// convert pi control to VelocityHeight control
			hal_quadrotor::VelocityHeight req_VelocityHeight;
			req_VelocityHeight.request.dx = sp[2];
			req_VelocityHeight.request.dy = sp[3];
			req_VelocityHeight.request.z = takeoffz;
			req_VelocityHeight.request.yaw = 0.;
		
			if(!srvVelocityHeight.call(req_VelocityHeight)){
				ROS_FATAL("NO VELHEIGHT!!");
			} else {
				ROS_INFO("VelocityHeight command sent!!");
				state_FSM = OTHER;
			}	
			// END PI
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
//	//string ip = pt.get<string>("ip");
//	//int port = pt.get<int>("port");
//	//string taskfile = pt.get<string>("taskfile");
//	double dt = pt.get<double>("dt");
//	double R = pt.get<double>("R");
//	int dtperstep = pt.get<int>("dtperstep");
//	int H = pt.get<int>("H");
//	double nu = pt.get<double>("nu");
//	double T = pt.get<double>("T");
//	int seed = pt.get<int>("seed");
//	int N = pt.get<int>("N");
//	int units = pt.get<int>("units");
//	int dimUAVx = pt.get<int>("dimUAVx");
//	int dimUAVu = pt.get<int>("dimUAVu");
//
//	takeoffx = pt.get<double>("takeoffx");
//	takeoffy = pt.get<double>("takeoffy");
//	takeoffz = pt.get<double>("takeoffz");
//	idunit = pt.get<string>("idunit");
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
//		pi.Init(dimUAVx, dimUAVu,	seed,	N);

		// Initial state X should be input as well (don't forget tpo convert)
		// create instance of PIController
		pi.Init(pt);//prop);

		double T = pt.get<double>("T");		

	} catch (int e) {
		cout << "Exception" << e << endl;
	}

  // Initialise the ROS client
	string nodestr = string("example") + idunit;
  ros::init(argc, argv, nodestr.c_str()); //"example");

  // Create a node handle, which this C code will use to bind to the ROS server
  ros::NodeHandle n;

  // Create a client for interacting with the Simulator insert and Resume services
  ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
  ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
  ros::ServiceClient srvPause  = n.serviceClient<sim::Pause> ("/simulator/Pause");

  // Subscribe to the state of the quadrotor
  string estimate = string("/hal/") + idunit + string("/Estimate");
	ros::Subscriber topState = n.subscribe(estimate.c_str(), 1000, StateCallback);

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
  msgInsert.request.model_name = idunit;//"UAV0";
  msgInsert.request.model_type = "model://hummingbird";
  
  // Call the client with this message
  if (!srvInsert.call(msgInsert))
  {
    ROS_FATAL("Failed to insert a hummingbird quadrotor into the simulator");
    return 1;
  }

  // You should now see the quadrotor
  ROS_INFO("Successfully inserted the model into the world");

	sleep(2);
  // Create a resume message, which takes no arguments
  sim::Resume msgResume;

  // Call the client with this message
  if (!srvResume.call(msgResume))
  {
    ROS_FATAL("Failed to resume the simulator");
    return 1;
  }

  // rosservice call /hal/UAV0/controller/Takeoff 5
	string takeoff_cmd = string("hal/") + idunit + string("/controller/Takeoff");
	srvTakeOff = n.serviceClient<hal_quadrotor::Takeoff>(takeoff_cmd.c_str()); //"/hal/UAV0/controller/Takeoff");
	
	string velheight_cmd = string("hal/") + idunit + string("/controller/VelocityHeight");
	srvVelocityHeight = n.serviceClient<hal_quadrotor::VelocityHeight>(velheight_cmd.c_str()); //"/hal/UAV0/controller/VelocityHeight");
	
//	sleep(10);
//  // rosservice call /hal/UAV0/controller/Hover
//	hal_quadrotor::Hover req_Hover;
//	ros::ServiceClient srvHover = n.serviceClient<hal_quadrotor::Hover>("/hal/UAV0/controller/Hover");
//	if(!srvHover.call(req_Hover)){
//		ROS_FATAL("NO HOVER!!");
//		return 1;
//	} else {
//		ROS_INFO("Hovering!!");
//	}
//	
//	sleep(10);
//  // rosservice call /hal/UAV0/controller/WayPoint
//	hal_quadrotor::Waypoint req_Waypoint;
//	req_Waypoint.request.x = 1.0;
//	req_Waypoint.request.y = 3.0;
//	req_Waypoint.request.z = 5.0;
//	req_Waypoint.request.yaw = 0.;
//
//	ros::ServiceClient srvWaypoint = n.serviceClient<hal_quadrotor::Waypoint>("/hal/UAV0/controller/Waypoint");
//	if(!srvWaypoint.call(req_Waypoint)){
//		ROS_FATAL("NO WAYPOINT!!");
//		return 1;
//	} else {
//		ROS_INFO("Waypointing!!");
//	}
	
  // Keep going until ctl+c is pressed
  ros::spin();

  // Success!
  return 0;
}
