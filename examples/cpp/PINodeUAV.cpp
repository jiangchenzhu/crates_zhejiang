#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 

#include <ros/ros.h>
// Quadrotor controllers
#include <hal_quadrotor/control/Takeoff.h>
#include <hal_quadrotor/control/VelocityHeight.h>
//#include <hal_quadrotor/control/Waypoint.h>
//#include <hal_quadrotor/control/Hover.h>


// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

#include "pi/global.h"

#define EPS_POS			5e-1	// epsilon position
#define EPS_VEL			1e-3	// epsilon position

typedef enum { IDLE, TAKEOFF, VELHEIGHT, HOVER, WAYPOINT, OTHER } FSM;

// TODO: better way to get parameters in the callback instead of global vars
ros::ServiceClient srvTakeOff; 
ros::ServiceClient srvVelocityHeight; 
FSM state_FSM = IDLE;

using namespace std;

int fd; 				// FIFO descriptor
double takeoffx; 
double takeoffy; 			
double takeoffz; 
string idunit;	// id of the UAV

// compares two states 
bool sameState(const hal_quadrotor::State::ConstPtr& s1, const hal_quadrotor::State &s2)
{
	double dpos = sqrt((s1->x-s2.x)*(s1->x-s2.x) + (s1->y-s2.y)*(s1->y-s2.y) + (s1->z-s2.z)*(s1->z-s2.z));
	double dvel = sqrt((s1->u-s2.u)*(s1->u-s2.u) + (s1->v-s2.v)*(s1->v-s2.v));
 	ROS_INFO("dpos: [%f %f]", dpos, dvel);
	return ((dpos < EPS_POS) && (dvel < EPS_VEL));
}

// Callback for quadrotor state
// receives state of the UAV
// writes state in the FIFO 
void StateCallback(const hal_quadrotor::State::ConstPtr& msg)
{
//	cout << "writing FIFO" << endl;
//	string msgfifo = getState(msg);
//	int nw = write(fd, msgfifo.c_str(), SIZE_MSG);
//	if (nw<0)	cerr << "write errror" << endl;
//	else cout << "state written to FIFO " << nw << " bytes" << endl;

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
		default: {
  		ROS_INFO("pos: [%f,%f,%f]", msg->x, msg->y, msg->z);
		} break;
	}
	
}


// Main entry point of application
int main(int argc, char **argv)
{
	//////////////////////////////////////////////
	// read parameters from xml file
	if (argc!=2) {
		cout << "requires xml_file as argument" << endl;
		exit(-1);
	}
	ifstream is(argv[1]);
	using boost::property_tree::ptree;
	ptree pt;
	if (!is.is_open()) {
		cout << "file not found" << endl;
		exit(-1);
	}
	read_xml(is, pt);
	
	// task parameters
	takeoffx = pt.get<double>("takeoffx");
	takeoffy = pt.get<double>("takeoffy");
	takeoffz = pt.get<double>("takeoffz");
	idunit = pt.get<string>("idunit");
	
	//////////////////////////////////////////////
	// open FIFO for writing states
	pid_t pid;
	string myfifo("/tmp/pi_fifo");
	if (access(myfifo.c_str(), F_OK) == -1) {
		cerr << "fifo does not exist, launch PIROS first" << endl;
		return EXIT_FAILURE;
	}
	if ((fd = open(myfifo.c_str(), O_WRONLY))<0) {
		cout << "open failed" << endl;
		return EXIT_FAILURE;
	}
	else cout << "Opened successfully" << endl;

	cout << "writing FIFO" << endl;
	string msgfifo = string("XXXXX");
	int nw = write(fd, msgfifo.c_str(), SIZE_MSG);
	if (nw<0)	cerr << "write errror" << endl;
	else cout << "state written to FIFO " << nw << " bytes" << endl;


//	cout << "blocked in write" << endl;
//	cout << write(fd, idunit.c_str(), sizeof(idunit.c_str()) << endl;
//	cout << "state written to FIFO" << endl;

	exit(-1);
	
	//////////////////////////////////////////////
	// initialize ROS
	string nodestr = string("PINode") + idunit;
  ros::init(argc, argv, nodestr.c_str());
	ros::NodeHandle n;

	// Create a client for interacting with the Simulator insert and Resume services
  ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
  ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
  ros::ServiceClient srvPause  = n.serviceClient<sim::Pause> ("/simulator/Pause");

  // Subscribe to the state of the quadrotor
	string estimate = string("/hal/") + idunit + string("/Estimate");
  cout << estimate << endl;
	ros::Subscriber topState = n.subscribe(estimate.c_str(), 1000, StateCallback);

  // Create a resume message, which takes no arguments
  sim::Pause msgPause;

  // Call the client with this message
  if (!srvPause.call(msgPause)) {
    ROS_FATAL("Failed to pause the simulator");
    return 1;
  }

  // Create a message instructing a quadrotor UAV0 to be inserted 
  sim::Insert msgInsert;
  msgInsert.request.model_name = idunit;
  msgInsert.request.model_type = "model://hummingbird";
  
  // Call the client with this message
  if (!srvInsert.call(msgInsert)) {
    ROS_FATAL("Failed to insert a hummingbird quadrotor into the simulator");
    return 1;
  }

  // You should now see the quadrotor
  ROS_INFO("Successfully inserted the model into the world");

	sleep(5);
  ROS_INFO("AWAKE");
  // Create a resume message, which takes no arguments
  sim::Resume msgResume;

  // Call the client with this message
  if (!srvResume.call(msgResume)) {
    ROS_FATAL("Failed to resume the simulator");
    return 1;
  }

  // takeoff and velheight commands 
	string takeoff_cmd = string("hal/") + idunit + string("/controller/Takeoff");
	srvTakeOff = n.serviceClient<hal_quadrotor::Takeoff>(takeoff_cmd.c_str());
	
	string velheight_cmd = string("hal/") + idunit + string("/controller/VelocityHeight");
	srvVelocityHeight = n.serviceClient<hal_quadrotor::VelocityHeight>(velheight_cmd.c_str()); 
	
	// Keep going until ctl+c is pressed
  ros::spin();

  // Success!
  return 0;

}
