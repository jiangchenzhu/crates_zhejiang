#!/bin/bash

# Make the system aware of ROS
source /home/complacs/crates/devel/setup.bash

# Start RTK listening for base station raw data (and logging of course)
/home/complacs/RTKLIB/apps/grtkrcv/gcc/rtkrcv -o /home/complacs/quadrotor.conf -s

# Launch the quadrotor
roslaunch platform_asctec pelican.launch id:=`hostname`