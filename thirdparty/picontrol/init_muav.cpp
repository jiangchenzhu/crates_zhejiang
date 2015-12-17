// General ROS functionality
#include <ros/ros.h>

// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

//#include "pi/global.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 

using namespace std;

vector<string> idunits;
int units;

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
		  ROS_FATAL("xml config file required!"); 
		  return 1;
		}
		read_xml(is, pt);
		
		// task parameters (only units and idunits are required)
		units = pt.get<int>("units");
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("idunits")) {
			idunits.push_back(v.second.get<string>(""));
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

  // Success!
  return 0;
}
