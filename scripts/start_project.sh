#!/bin/bash

# Build the catkin_ws
cd $(pwd)/../../.. ; catkin_make

# Launch the nodes
xterm  -e "source devel/setup.bash; roslaunch rll_planning_project moveit_planning_execution.launch" &
sleep 10
xterm  -e "source devel/setup.bash; roslaunch rll_planning_project planning_iface.launch" &
sleep 4	
xterm  -e "source devel/setup.bash; roslaunch rll_planning_project path_planner.launch" &
sleep 4
xterm  -e "source devel/setup.bash; roslaunch rll_project_runner run_project.launch" &

# Termiante the project
echo "Press Enter key to terminate the project"
read -s -n 1 key
if [[ $key = "" ]]; then 
	killall -e xterm
	echo "Project terminated!"
fi

