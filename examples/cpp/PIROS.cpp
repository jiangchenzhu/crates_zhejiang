#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>

#include "pi/global.h"


using namespace std;

// define callback for PI control
	// receives a copy of simplified joint state
	// (predicts next state to apply control)
	// calls ComputeControl
	// translates control to VelHeight command
	// distributes VelHeight control commands to the UAVs

// Main entry point of application
int main(int argc, char **argv)
{
	pid_t pid;
	string myfifo("/tmp/pi_fifo");

	// initialize ROS
	ros::init(argc, argv, "ROSPI");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

  // Create the child process
  pid = fork();
  if (pid == (pid_t) 0) {

		// child process
		// open FIFO for reading states
		if(access(myfifo.c_str(), F_OK) == -1 ) {
			if ( mkfifo(myfifo.c_str(), 0666) < 0 ) {
	      cerr << "mkfifo failed" << endl;
	      return EXIT_FAILURE;
	    }
			cout << "fifo created" << endl;
		}
    else if(access(myfifo.c_str(), F_OK) == 0) {
      cout << "fifo exists" << endl;
		}

		// listen and update shared memory joint state object
		int count = 0;
		int fd;
    cout << "Blocking in Trying to open fifo for READ" << endl;
	  if ((fd = open(myfifo.c_str(), O_RDONLY))<0) { // | O_NONBLOCK 
			cout << "open failed" << endl;
			return EXIT_FAILURE;
		}
		else cout << "Opened successfully" << endl;

		cout << "Starting reading loop" << endl;
		bool fin = false;
		while (!fin) {
	    
			// read from FIFO
			cout << "Reading from FIFO " << count << endl;
			int nr = 0;
			char buf[SIZE_MSG];
	    if ((nr=read(fd, buf, SIZE_MSG))<0) {
				cerr << "error reading" << endl;
			}
			if (nr>0)
		    cout << "Received " << nr << " bytes : " << buf << endl;
			
			ros::spinOnce(); 
			loop_rate.sleep();
			++count;
		}

    // remove the FIFO
   	close(fd);
		cout << "fifo closed" << endl;
    unlink(myfifo.c_str());
		cout << "fifo removed" << endl;
		exit(0);
  }
  else if (pid < (pid_t) 0) {
      /* The fork failed. */
      cerr << "fork failed" << endl;
      return EXIT_FAILURE;
  }
  else {
		// parent process
		// create shared memory joint state object
		// initialize PIController
		// advertise topic PI
		// subscribe topic PI
		// fork
		// loop 
			// copies the latest shared memory joint state object to a simplified
			// publishes event with simplified joint state
      return EXIT_SUCCESS;
  }


}
