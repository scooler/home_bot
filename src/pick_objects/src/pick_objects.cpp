#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1;

  // set up the frame parameters
  // goal.target_pose.header.frame_id = "base_link";
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal1.target_pose.pose.position.x = 2.15302252769;
  goal1.target_pose.pose.position.y = -6.16027069092;
  goal1.target_pose.pose.orientation.w = 1.0;



// --- # Point 1 - next to the shelf
// header:
//   seq: 3
//   stamp:
//     secs: 217
//     nsecs: 267000000
//   frame_id: "map"
// point:
//   x: 2.15302252769
//   y: -6.16027069092
//   z: -0.00143432617188

// --- Point 2 - on the other side of the room
// header:
//   seq: 4
//   stamp:
//     secs: 247
//     nsecs: 892000000
//   frame_id: "map"
// point:
//   x: -4.97087669373
//   y: -1.49952840805
//   z: -0.00143432617188

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Moving to pickup area");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();



  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

    ROS_INFO("YAY ! Reached the pickup spot");
    ros::Duration(5.0).sleep();

    move_base_msgs::MoveBaseGoal goal2;
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();
    goal2.target_pose.pose.position.x = -4.97087669373;
    goal2.target_pose.pose.position.y = -1.49952840805;
    goal2.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Moving to dropoff area");
    ac.sendGoal(goal2);
    ac.waitForResult();
  }
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}