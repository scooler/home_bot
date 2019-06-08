#!/usr/bin/env zsh

if [[ `pwd` == *"scripts" ]]; then
  cd ..
fi
echo $(pwd)

xterm  -e  "source devel/setup.zsh; roscore" &

sleep 4
xterm  -e  "source devel/setup.zsh; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/my_robot/worlds/world4.world" &


sleep 5
xterm  -e  "source devel/setup.zsh; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/maps/world43.yaml" &

sleep 5
xterm  -e  " source devel/setup.zsh; roslaunch turtlebot_rviz_launchers view_navigation.launch" &


sleep 5
xterm  -e  " source devel/setup.zsh; rosrun add_markers add_markers" &
