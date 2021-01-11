// Code for testing the setup of Franka Emika Panda robot with MoveIt
// Many of the codes are from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
// Author: Hongtao Wu
// Jan 11, 2021

#include <iostream>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#define PI 3.1415926

void printRobotState(const std::vector<double>& joint_group_positions)
{
  for(int i=0; i<joint_group_positions.size(); i++)
    std::cout << "Joint[" << i << "] = " << joint_group_positions[i] << std::endl;
}

void printRobotPose(
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const geometry_msgs::PoseStamped& pose_stamped)
{
  std::cout << "End Effector Link: " << move_group.getEndEffectorLink().c_str() << std::endl;
  std::cout << "Robot Position x: " << pose_stamped.pose.position.x << std::endl;
  std::cout << "Robot Position y: " << pose_stamped.pose.position.y << std::endl;
  std::cout << "Robot Position z: " << pose_stamped.pose.position.z << std::endl;
  std::cout << "Robot Orientation w: " << pose_stamped.pose.orientation.w << std::endl;
  std::cout << "Robot Orientation x: " << pose_stamped.pose.orientation.x << std::endl;
  std::cout << "Robot Orientation y: " << pose_stamped.pose.orientation.y << std::endl;
  std::cout << "Robot Orientation z: " << pose_stamped.pose.orientation.z << std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_move_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("test", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("test", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("test", "Goal joint tolerance: %f", move_group.getGoalJointTolerance());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;
  joint_group_positions[1] = -PI/4;
  joint_group_positions[2] = 0.0;
  joint_group_positions[3] = -2*PI/3;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = PI/3;
  joint_group_positions[6] = PI/4;

  move_group.setJointValueTarget(joint_group_positions);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("test", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot to joint goal");

  // Robot will move
  move_group.move();
  ROS_INFO_NAMED("test", "Finish moving to start joint goal"); 

  // Check the current joint position and compare with the goal
  joint_group_positions = move_group.getCurrentJointValues();
  printRobotState(joint_group_positions);

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.0;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 1.0;
  target_pose1.orientation.z = 0.0;

  target_pose1.position.x = 0.28;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("test", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the real robot");

  move_group.move();
  ROS_INFO_NAMED("test", "Finishing moving to the goal configuration");

  // Check the current pose of the end effector and compare with the goal
  geometry_msgs::PoseStamped goal_pose1;
  goal_pose1 = move_group.getCurrentPose();
  printRobotPose(move_group, goal_pose1);

  ros::shutdown();
  return 0;
}