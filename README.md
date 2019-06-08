# Home Bot

A simple robot - using gmapping it SLAM-s the world. Maps made with that are in maps.


## Build

To build just run
```
catkin_make
```

In the main directory. That should create devel directory. So then `source devel/setup.bash` (or `.zsh` if that's your fancy ;) ).


## Run

Scripts are all in `scripts` directory. (prerequisites - even thou they're `.sh` - I've written them with zsh - I use it and I'm lazy ;P )
- launch.sh - is just a dummy
- test_slam.sh - allows you to navigate the robot with keyboard and SLAMs the world - showing the map in rviz, and gazebo is visible too (by default)
- test_navigation.sh - lets you navigate the robot with rviz commands. It will find a path, and get there - if it can. Watch out for the narrow doors :D
- pick_objects.sh - uses `pick_objects` node in `pick_objects` package. It drives the robot to 2 predefined spots ("pickup" & "dropoff")
- add_marker.sh - shows a cube in rviz in the "pickup" spot, and after 5s makes it disappear, and shows it in "dropoff"
- home_service.sh - joins the two such that `add_markers` listens to topic that `pick_objects` publishes to. Once pick_objects says he reached pickup - the cube disapears. And once it gets to dropoff (letting the `add_markers` know) - the cube shows up.

