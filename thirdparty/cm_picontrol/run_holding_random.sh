#!/bin/bash
for h in 1 3 5 7; do
	for i in `seq 1 10`; do
		roslaunch sim sw.launch &
		sleep 5
		./build/init_muav ./pi_UAV_joint_H$h.xml $i ; ./build/main_muav ./pi_UAV_joint_H$h.xml &
		sleep 40 
		./killros.sh
		sleep 15
	done
done 
