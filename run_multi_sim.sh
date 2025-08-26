#!/bin/bash

# Path to swarm-ws workspace
ROS2_WS=~/swarm_ws/install

# Helper function to open gnome-terminal, source ROS2 + swarm_ws, and execute a ros2 run node
run_ros(){
	  local title=$1
	  local cmd=$2
	  
	  gnome-terminal\
  	  --title="$title"\
	  -- bash -c "
    	  source /opt/ros/humble/setup.bash &&\
	  source $ROS2_WS/setup.bash &&\
	  source $ROS2_WS/local_setup.bash &&\
	  sleep 1 &&\
	  $cmd;\
	  exec bash"
	  }

# Helper function to open gnome-terminal, and spawn x500 models and px4 instances
spawn_x500(){
	    local title=$1
	    local cmd=$2
	    local pose=$3
	    local id=$4
	
	gnome-terminal\
	--title="$title"\
	-- bash -c "cd ~/PX4-Autopilot &&\
	$cmd &&\
	PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$pose\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $id;\
	exec bash" 
	}

# Spawn X500_1 
spawn_x500 "PX4 instance 1" "make px4_sitl" "1,1,0.1" "1"
sleep 5

# Spawn X500_2 
spawn_x500 "PX4 instance 2" "PX4_GZ_STANDALONE=1" "0,0,0.1" "2"
sleep 3


# Spawn X500_3 
spawn_x500 "PX4 instance 3" "PX4_GZ_STANDALONE=1" "0,1,0.1" "3"
sleep 3


# Spawn X500_4 
spawn_x500 "PX4 instance 4" "PX4_GZ_STANDALONE=1" "0,2,0.1" "4"
sleep 5


# Run MicroXRCEAgent 
gnome-terminal --title="MicroXRCEAgent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash;"
sleep 5

run_ros "Drone 1 Offboard" "ros2 run swarm_control offboard_takeoff --ros-args -p drone_ID:=1"
run_ros "Drone 2 Offboard" "ros2 run swarm_control offboard_takeoff --ros-args -p drone_ID:=2"
run_ros "Drone 3 Offboard" "ros2 run swarm_control offboard_takeoff --ros-args -p drone_ID:=3"
run_ros "Drone 4 Offboard" "ros2 run swarm_control offboard_takeoff --ros-args -p drone_ID:=4"
