#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("manipulator");
  arm.setPoseReferenceFrame("base_link");
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
      "/gripper/gripper_cmd",
      "true");
  gripper.waitForServer();

  // Prepare
  ROS_INFO("Moving to prepare pose");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;

  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to prepare pose");
    return 1;
  }

  // Open gripper
  ROS_INFO("Opening gripper");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.1;
  gripper.sendGoal(goal);
  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper open action did not complete");
    return 1;
  }

  // Approach
  ROS_INFO("Executing approach");
  pose.pose.position.z = 0.05;
  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to grasp pose");
    return 1;
  }

  // Grasp
  ROS_INFO("Grasping object");
  goal.command.position = 0.015;
  gripper.sendGoal(goal);
  finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper close action did not complete");
    return 1;
  }

  // Retreat
  ROS_INFO("Retreating");
  pose.pose.position.z = 0.1;
  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to retreat pose");
    return 1;
  }

  ros::shutdown();
  return 0;
}