#!/bin/bash
for i in `seq 5`; do
	for n in 100 500 5000; do
		for u in 2 4 6 8 10 12; do
			roslaunch sim sw.launch &
			sleep 5
			./build/init_muav ./pi_UAV_joint_u${u}_n${n}.xml $i; ./build/main_muav ./pi_UAV_joint_u${u}_n${n}.xml &
			sleep 120 
			./killros.sh
		sleep 15
		done
	done
done 
