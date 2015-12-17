#!/bin/bash

# Number of compilation threads (reduce if there are problems with vm calculations)
NT=2

# Add to apt
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

# Add the key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update the libraries
sudo apt-get update

# Install base libraries
sudo apt-get -y install libbullet2.82-dev libsdformat2-dev

# Save the base directory
BASEDIR=${PWD}

# Install gazebo
cd $BASEDIR
if [[ ! -d $BASEDIR/gazebo ]]; then
	hg clone https://bitbucket.org/osrf/gazebo
fi
cd gazebo
hg up gazebo4_4.0.1
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_TESTS_COMPILATION:BOOL=False -DCMAKE_BUILD_TYPE=Release ../
make -j$NT
sudo make install

# Create a home directory or gazebo wont start
mkdir -p ~/.gazebo/models