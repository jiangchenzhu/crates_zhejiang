#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/time_synchronizer.h>
#include <hal_quadrotor/State.h>
#include <hal_quadrotor/Control.h>

int main(int argc, char* argv[])
{
  // We are in science here :)
  std::cout.precision(16);

  // Argument check
  if (argc < 3)
  {
    std::cout << "Usage: " << argv[0] << " <file> <topic>" << std::endl;
    return 1;
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string(argv[2]));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic()=="/hal/UAV0/Estimate")
    {
      hal_quadrotor::State::ConstPtr state = m.instantiate<hal_quadrotor::State>();
      std::cout << state->t << ",";
      std::cout << state->x << ",";
      std::cout << state->y << ",";
      std::cout << state->z << ",";
      std::cout << state->roll << ",";
      std::cout << state->pitch << ",";
      std::cout << state->yaw << ",";
      std::cout << state->u << ",";
      std::cout << state->v << ",";
      std::cout << state->w << ",";
      std::cout << state->p << ",";
      std::cout << state->q << ",";
      std::cout << state->r << ",";
      std::cout << state->thrust << std::endl;
    }
    if (m.getTopic()=="/hal/UAV0/Control")
    {
      hal_quadrotor::Control::ConstPtr control = m.instantiate<hal_quadrotor::Control>();
      std::cout << control->t << ",";
      std::cout << control->pitch << ",";
      std::cout << control->roll << ",";
      std::cout << control->throttle << ",";
      std::cout << control->yaw << std::endl;
    }
  }

  bag.close();
  
  return 0;

}