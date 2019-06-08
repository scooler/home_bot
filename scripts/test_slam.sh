#!/usr/bin/env zsh

# sleep 5
if [[ `pwd` == *"scripts" ]]; then
  cd ..
fi
echo $(pwd)

xterm  -e  "source devel/setup.zsh; roscore" &

sleep 4
xterm  -e  "source devel/setup.zsh; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/my_robot/worlds/world4.world" &

sleep 5
xterm  -e  " source devel/setup.zsh; roslaunch turtlebot_gazebo gmapping_demo.launch" &
# sleep 5
# xterm  -e  " source devel/setup.zsh; roslaunch gmapping slam_gmapping_pr2.launch" &

sleep 5
xterm  -e  " source devel/setup.zsh; roslaunch turtlebot_teleop keyboard_teleop.launch" &

sleep 5
xterm  -e  " source devel/setup.zsh; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
