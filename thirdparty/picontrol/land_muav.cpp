// General ROS functionality
#include <ros/ros.h>

#include <boost/regex.hpp>

// Quadrotor controllers
#include <hal_quadrotor/control/Hover.h>

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

			boost::cmatch mr;
			if (boost::regex_match(i->name.c_str(), mr, boost::regex("^/hal/(.+)/(.+)/(.+)/Estimate$")))
			{
				string land_cmd = string("/hal/") + mr[1].str() + "/" + mr[2].str() + "/" + mr[3].str() + string("/controller/Hover");
				ROS_INFO("land command %s", land_cmd.c_str());
				ros::ServiceClient srvLand = n.serviceClient<hal_quadrotor::Hover>(land_cmd.c_str());
				hal_quadrotor::Hover msgLand; 
				if(!srvLand.call(msgLand))
					ROS_FATAL("NO Land!!");
			}
		}
	}
	

  // Success!
  return 0;
}
