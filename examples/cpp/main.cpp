// General ROS functionality
#include <ros/ros.h>

// Quadrotor services
#include <hal_quadrotor/Takeoff.h>

// Main entry point of application
int main(int argc, char **argv)
{
  // Initialise the ROS client
  ros::init(argc, argv, "example");

  // Create a node handle, which this C code will use to bind to the ROS server
  ros::NodeHandle n;

  // Create a client for interacting with the Simulator insert and Resume services
  ros::ServiceClient srvTakeoff = n.serviceClient<hal_quadrotor::Takeoff>("/hal/quadrotor/UAV0/controller/Takeoff");

  // Create a message instructing a quadrotor UAV0 to be inserted in to the world
  hal_quadrotor::Takeoff msgTakeoff;
  msgTakeoff.request.altitude = 5;
  
  // Call the client with this message
  if (!srvTakeoff.call(msgTakeoff))
  {
    ROS_FATAL("Failed to insert a hummingbird quadrotor into the simulator");
    return 1;
  }

  // Keep going until ctl+c is pressed
  ros::spin();

  // Success!
  return 0;
}
