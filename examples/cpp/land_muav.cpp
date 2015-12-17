// General ROS functionality
#include <ros/ros.h>

// Quadrotor controllers
#include <hal_quadrotor/control/Land.h>

using namespace std;

// Main entry point of application
int main(int argc, char **argv)
{
	// Initialise the ROS client
	string nodestr = string("land_muav");
  ros::init(argc, argv, nodestr.c_str()); 

  // Create a node handle, which this C code will use to bind to the ROS server
  ros::NodeHandle n;

	// Try and get a list of topics currently being braodcast
	ros::master::V_TopicInfo topics;
	vector<string> names;
	vector<ros::Subscriber> top_state;

	// Iterate over the topics
	if (ros::master::getTopics(topics)) { 
		for (ros::master::V_TopicInfo::iterator i = topics.begin(); i != topics.end(); i++) {

			// Check if type corresponds to a hal_quadrotor/State
			//if (!strcmp("hal_quadrotor/State", i->datatype.c_str())) {
			if (i->name.find("Estimate")!=string::npos) {
				
				// Unit found Make it land
				string land_cmd = i->name.substr(0, i->name.find("Estimate")-1) + string("/controller/Land");
				ROS_INFO("land command %s", land_cmd.c_str());
				ros::ServiceClient srvLand = n.serviceClient<hal_quadrotor::Land>(land_cmd.c_str());
				hal_quadrotor::Land msgLand; 
				if(!srvLand.call(msgLand))
					ROS_FATAL("NO Land!!");
			}
		}
	}
	

  // Success!
  return 0;
}
