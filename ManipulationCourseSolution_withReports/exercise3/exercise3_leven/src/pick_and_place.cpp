/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Modified on 08/01/20
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */
#include <exercise3/functions.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(2);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  spinner.start();
  
  // Wait for the simulator to come online and then we reset it
  std_srvs::Empty srv_reset;
  ros::service::waitForService("/lumi_mujoco/reset");
  ros::service::call("/lumi_mujoco/reset", srv_reset);
  
  // read poses from tf
  geometry_msgs::TransformStamped base_to_pick_transform;
  geometry_msgs::TransformStamped base_to_place_transform;
  base_to_pick_transform = tfBuffer.lookupTransform("base_link","pick", ros::Time(0), ros::Duration(1));
  base_to_place_transform = tfBuffer.lookupTransform("base_link","place", ros::Time(0), ros::Duration(1));

  // convert poses to Eigen
  Eigen::Isometry3d b_t_pick;
  Eigen::Isometry3d b_t_place;
  tf::transformMsgToEigen(base_to_pick_transform.transform, b_t_pick);
  tf::transformMsgToEigen(base_to_place_transform.transform, b_t_place);
  
  // Load MoveGroup interface and moveit visual tools
  moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
  moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
  moveit_visual_tools::MoveItVisualTools vis("base_link");
  vis.loadMarkerPub(true);
  vis.deleteAllMarkers();
  vis.trigger();
  
  // Get Start state
  const robot_state::RobotStatePtr state_ptr = g_arm.getCurrentState(10.0);
  if (!state_ptr) {
    ROS_ERROR_STREAM("Cannot get current state");
    return -1;
  }
  robot_state::RobotState state = *state_ptr;
  
  const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(
    "lumi_arm"); // joint model group used for IK computation
  
  const std::string ee_link = "lumi_ee"; // Name of the end effector link
  const Eigen::Isometry3d arm_to_ee =
    state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
    state.getGlobalLinkTransform(
      ee_link); // Transformation from base link to end effector link
  
    // compute required poses and visualize them
    Eigen::Isometry3d pre_grasp_pose_eigen = state.getGlobalLinkTransform("base_link") * b_t_pick *
      Eigen::Translation3d(0.0, 0.0, 0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    b_t_pick =  state.getGlobalLinkTransform("base_link") * b_t_pick * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    b_t_place = state.getGlobalLinkTransform("base_link") * b_t_place * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  
  // visualize the frames
  vis.publishAxis(pre_grasp_pose_eigen);
  vis.publishAxis(b_t_pick);
  vis.publishAxis(b_t_place);

  vis.trigger();
  
  std::vector<moveit_msgs::RobotTrajectory> trajectories;
  
  // planning from START to PREGRASP
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, pre_grasp_pose_eigen, ee_link)) {
    ROS_ERROR_STREAM("Cannot set arm position with IK");
    return -1;
  }
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  } 
  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);
  
  // cartesian pose from PREGRASP to GRASP
  g_arm.setStartState(state);
  geometry_msgs::Pose grasp_pose;
  tf::poseEigenToMsg(b_t_pick * arm_to_ee.inverse(), grasp_pose);

  moveit_msgs::RobotTrajectory rtraj;
  const double d = g_arm.computeCartesianPath({grasp_pose}, 0.01, 1.4, rtraj);
  if (d < 0.99) {
    ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
    return -1;
  }
  trajectories.push_back(rtraj);
  
  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
  
  // cartesian pose from GRASP back to PREGRASP
  g_arm.setStartState(state);
  geometry_msgs::Pose pre_grasp_pose;
  tf::poseEigenToMsg(pre_grasp_pose_eigen * arm_to_ee.inverse(), pre_grasp_pose);
  g_arm.computeCartesianPath({pre_grasp_pose}, 0.01, 1.4, rtraj);

  trajectories.push_back(rtraj);
  
  state.setVariablePositions(
    trajectories.back().joint_trajectory.joint_names,
    trajectories.back().joint_trajectory.points.back().positions);
  
  // planning from GRASP to PLACE
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, b_t_place, ee_link)) {
    ROS_ERROR_STREAM("Cannot set arm position with IK");
    return -1;
  }
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  } 

  // Visualise all trajectories
  for (const moveit_msgs::RobotTrajectory &t : trajectories) {
    vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
  }
  vis.trigger();

  if (askContinue()) {
    g_hand.setStartStateToCurrentState();
    
    // Execute trajectory to PRE_GRASP
    g_arm.execute(trajectoryToPlan(trajectories[0]));
    ros::Duration(1.0).sleep();
    // OPEN GRIPPER
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true)));
    ros::Duration(1.0).sleep();
    // Execute trajectory to GRASP
    g_arm.execute(trajectoryToPlan(trajectories[1]));
    ros::Duration(1.0).sleep();
    // CLOSE GRIPPER
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, false)));
    ros::Duration(1.0).sleep();
    // Execute trajectory back to PRE_GRASP
    g_arm.execute(trajectoryToPlan(trajectories[2]));
    ros::Duration(1.0).sleep();
    // Execute trajectory to PLACE
    g_arm.execute(trajectoryToPlan(trajectories[3]));
    ros::Duration(1.0).sleep();
    // OPEN GRIPPER
    g_hand.execute(trajectoryToPlan(getGripperTrajectory(g_hand, true)));
  } else {
    ROS_ERROR_STREAM("Execution of trajectory is CANCELLED.");
  }
  
  return 0;
}