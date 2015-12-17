// General ROS functionality
#include <ros/ros.h>

// Simulator services
#include <sim/Insert.h>
#include <sim/Resume.h>
#include <sim/Pause.h>

// Quadrotor services
#include <hal_quadrotor/State.h>

double X[6];

// Callbacks for quadrotor state
void StateCallback0(const hal_quadrotor::State::ConstPtr& msg)
{
  ROS_INFO("UAV0 at t = %f at pos: [%f,%f,%f]", msg->t, msg->x, msg->y, msg->z);
  X[0] = msg->x;
  X[1] = msg->y;
  X[2] = msg->z;
}
void StateCallback1(const hal_quadrotor::State::ConstPtr& msg)
{
  ROS_INFO("UAV1 at t = %f at pos: [%f,%f,%f]", msg->t, msg->x, msg->y, msg->z);
  X[3] = msg->x;
  X[4] = msg->y;
  X[5] = msg->z;
}



// Main entry point of application
int main(int argc, char **argv)
{
  // Initialise the ROS client
  ros::init(argc, argv, "muav");

  // Create a node handle, which this C code will use to bind to the ROS server
  ros::NodeHandle n;

  // Create a client for interacting with the Simulator insert and Resume services
  ros::ServiceClient srvInsert = n.serviceClient<sim::Insert>("/simulator/Insert");
  ros::ServiceClient srvResume = n.serviceClient<sim::Resume>("/simulator/Resume");
  ros::ServiceClient srvPause  = n.serviceClient<sim::Pause> ("/simulator/Pause");

  // Subscribe to the state of the quadrotor
  ros::Subscriber topState0 = n.subscribe("/hal/UAV0/Estimate", 1000, StateCallback0);
  ros::Subscriber topState1 = n.subscribe("/hal/UAV1/Estimate", 1000, StateCallback1);

  // Create a resume message, which takes no arguments
  sim::Pause msgPause;

  // Call the client with this message
  if (!srvPause.call(msgPause))
  {
    ROS_FATAL("Failed to pause the simulator");
    return 1;
  }

  // Create messages instructing quadrotors UAV0 and UAV1 to be inserted in to the world
  sim::Insert msgInsert0, msgInsert1;
  msgInsert0.request.model_name = "UAV0";
  msgInsert0.request.model_type = "model://hummingbird";
  msgInsert1.request.model_name = "UAV1";
  msgInsert1.request.model_type = "model://hummingbird";
  
  // Call the client with this message
  if (!srvInsert.call(msgInsert0) || !srvInsert.call(msgInsert1) )
  {
    ROS_FATAL("Failed to insert the two hummingbird quadrotors into the simulator.");
    return 1;
  }

  // You should now see the quadrotor
  ROS_INFO("Successfully inserted the two uav into the world");

  // Create a resume message, which takes no arguments
  sim::Resume msgResume;

  // Call the client with this message
  if (!srvResume.call(msgResume))
  {
    ROS_FATAL("Failed to resume the simulator");
    return 1;
  }

  // You should now see the quadrotor take off. 
  ROS_INFO("Successfully resumed the simulation");

  // A sleep will take 1/.38 seconds. 
  ros::Rate loop_rate(.38);

  while (ros::ok())
  {
  	// Instead of sleep we use our PI controller
    loop_rate.sleep();

	// During sleep (or PI-control) all messages get buffered. A spin will cause the Callbacks to work through the buffered messages. 
    ros::spinOnce();

  	ROS_INFO("Joint state is: [%f, %f, %f],  [%f, %f, %f].", X[0], X[1], X[2], X[3], X[4], X[5]);

 
  }


  return 0;
}
