/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#ifndef EXERCISE4_FUNCTIONS_H
#define EXERCISE4_FUNCTIONS_H

#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <opencv2/core/core.hpp>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloudT::Ptr scene(new PointCloudT);

/** @brief Return true iff user want to execute trajectory. */
bool askContinue() {
  ROS_INFO_STREAM("Execute [y/N]?");
  const auto ch = getchar();
  getchar(); // capture the enter character
  if (ch != 'y' && ch != 'Y') {
    ROS_ERROR_STREAM("Execution canceled by user.");
    return false;
  }
  return true;
}

/** @brief Compute gripper trajectory
 *  @param open if true the trajectory will open the gripper; close otherwise
 *  @return empty trajectory on error */
moveit_msgs::RobotTrajectory
getGripperTrajectory(moveit::planning_interface::MoveGroupInterface &g_hand,
                     bool open) {
  auto state = g_hand.getCurrentState();
  if (!state) {
    ROS_ERROR_STREAM("Cannot get start state");
    return moveit_msgs::RobotTrajectory();
  }
  state->setVariablePosition("lumi_finger_joint1", open ? 0.04 : 0.0);
  if (!g_hand.setJointValueTarget(*state)) {
    ROS_ERROR_STREAM("Cannot set joint value target");
    return moveit_msgs::RobotTrajectory();
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!g_hand.plan(plan)) {
    ROS_ERROR_STREAM("Cannot plan to the target");
    return moveit_msgs::RobotTrajectory();
  }
  return plan.trajectory_;
}

/** @brief Plan to the given state. Return empty trajectory on error. */
moveit_msgs::RobotTrajectory
planToState(moveit::planning_interface::MoveGroupInterface &g_arm,
            const robot_state::RobotState &state) {
  if (!g_arm.setJointValueTarget(state)) {
    ROS_ERROR_STREAM("Cannot set joint value target");
    return moveit_msgs::RobotTrajectory();
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!g_arm.plan(plan)) {
    ROS_ERROR_STREAM("Cannot plan to the target");
    return moveit_msgs::RobotTrajectory();
  }
  return plan.trajectory_;
}

moveit::planning_interface::MoveGroupInterface::Plan
trajectoryToPlan(const moveit_msgs::RobotTrajectory &rtraj) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = rtraj;
  return plan;
}

void removePartOfPC(PointCloudT::Ptr pc, const double zMax, const double zMin) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointNT> extract;
  for (int i = 0; i < (*pc).size(); i++) {
    if (pc->points[i].z > zMax or pc->points[i].z < zMin) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(pc);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*pc);
}

geometry_msgs::TransformStamped createTranform(std::string headerID,
                                               std::string childID,
                                               Eigen::Matrix4f tMatrix) {
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = headerID;
  static_transformStamped.child_frame_id = childID;
  static_transformStamped.transform.translation.x = tMatrix(0, 3);
  static_transformStamped.transform.translation.y = tMatrix(1, 3);
  static_transformStamped.transform.translation.z = tMatrix(2, 3);
  Eigen::Matrix3f rotMat = tMatrix.block(0, 0, 3, 3);
  Eigen::Quaternionf q(rotMat);
  static_transformStamped.transform.rotation.x = q.x();
  static_transformStamped.transform.rotation.y = q.y();
  static_transformStamped.transform.rotation.z = q.z();
  static_transformStamped.transform.rotation.w = q.w();
  return static_transformStamped;
}

void callbackPC(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  if (scene->points.size() == 0) {
    ROS_INFO("Pointcloud was empty. Now populating it");
    scene->points.resize(cloud->size());
    scene->width = cloud->width;
    scene->height = cloud->height;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      scene->points[i].x = cloud->points[i].x;
      scene->points[i].y = cloud->points[i].y;
      scene->points[i].z = cloud->points[i].z;
    }
  }
}

void visualizeAlignment(const PointCloudT::Ptr scene,
                        const PointCloudT::Ptr object) {
  pcl::visualization::PCLVisualizer visu("Alignment");
  visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
  visu.addPointCloud(object, ColorHandlerT(object, 0.0, 0.0, 255.0),
                     "object_aligned");
  visu.spin();
}

int pickAndPlaceTheObject() {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped pick_pose, place_pose;
  try {
    pick_pose = tfBuffer.lookupTransform("base_link", "object", ros::Time(0),
                                         ros::Duration(5.0));
    place_pose = tfBuffer.lookupTransform("base_link", "place", ros::Time(0),
                                          ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR_STREAM(
        "Cannot find transformation to pick or place. What: " << ex.what());
    return -1;
  }

  Eigen::Isometry3d pick, place;
  tf::transformMsgToEigen(pick_pose.transform, pick);
  tf::transformMsgToEigen(place_pose.transform, place);
  pick(0, 0) = 1;
  pick(1, 1) = 1;
  pick(2, 2) = 1;
  pick(0, 1) = 0;
  pick(0, 2) = 0;
  pick(1, 0) = 0;
  pick(1, 2) = 0;
  pick(2, 0) = 0;
  pick(2, 1) = 0;
  // Subtract the height of the object which is 0.05 from the
  // z-direction of the pick pose. The reason for doing this is that
  // the pose estimation sets the grasping pose to the height of the
  // object and unless we subtract the object height the gripper will not
  // enclose the object and the picking will fail.
  pick(2, 3) -= 0.025;

  // Load MoveGroup interface and moveit visual tools
  moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
  moveit::planning_interface::MoveGroupInterface g_hand("lumi_hand");
  moveit_visual_tools::MoveItVisualTools vis("base_link");
  vis.loadMarkerPub(true);
  vis.deleteAllMarkers();
  vis.trigger();

  // Get Start state
  const auto state_ptr = g_arm.getCurrentState(10.0);
  if (!state_ptr) {
    ROS_ERROR_STREAM("Cannot get current state");
    return -1;
  }
  auto state = *state_ptr;

  const auto jmg = state.getJointModelGroup(
      "lumi_arm"); // joint model group used for IK computation
  const auto ee_link = "lumi_ee";
  const Eigen::Isometry3d arm_to_ee =
      state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
      state.getGlobalLinkTransform(ee_link);

  const auto rot_grip = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  const Eigen::Isometry3d pre_grasp =
      pick * Eigen::Translation3d(0.0, 0.0, 0.1) * rot_grip;
  const Eigen::Isometry3d grasp = pick * rot_grip;
  const Eigen::Isometry3d release = place * rot_grip;
  vis.publishAxis(pre_grasp);
  vis.publishAxis(grasp);
  vis.publishAxis(release);
  vis.trigger();

  std::vector<moveit_msgs::RobotTrajectory> trajectories;
  // plan to pre-grasp
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, pre_grasp, ee_link)) {
    ROS_ERROR_STREAM("Cannot set arm position with IK");
    return -1;
  }
  ROS_INFO("Made first path");
  trajectories.push_back(planToState(g_arm, state));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  }
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // open gripper
  g_hand.setStartState(state);
  trajectories.push_back(getGripperTrajectory(g_hand, true));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  }
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // cartesian path down
  g_arm.setStartState(state);
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(grasp * arm_to_ee.inverse(), pose);
  moveit_msgs::RobotTrajectory rtraj;
  const auto d = g_arm.computeCartesianPath({pose}, 0.01, 1.2, rtraj);
  if (d < 0.99) {
    ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
    return -1;
  }
  trajectories.push_back(rtraj);
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // close gripper
  g_hand.setStartState(state);
  trajectories.push_back(getGripperTrajectory(g_hand, false));
  if (trajectories.back().joint_trajectory.points.empty()) {
    return -1;
  }
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // cartesian path up

  g_arm.setStartState(state);
  tf::poseEigenToMsg(pre_grasp * arm_to_ee.inverse(), pose);
  const auto d1 = g_arm.computeCartesianPath({pose}, 0.01, 1.2, rtraj);
  if (d1 < 0.99) {
    ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
    return -1;
  }
  trajectories.push_back(rtraj);
  state.setVariablePositions(
      trajectories.back().joint_trajectory.joint_names,
      trajectories.back().joint_trajectory.points.back().positions);

  // plan to release pose
  g_arm.setStartState(state);
  if (!state.setFromIK(jmg, release, ee_link)) {
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

  // Visualise all trajectories
  for (const auto &t : trajectories) {
    vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
  }
  vis.trigger();

  if (askContinue()) {
    for (const auto &t : trajectories) {
      moveit::planning_interface::MoveGroupInterface::Plan p;
      p.trajectory_ = t;
      g_arm.execute(p);
      ros::Duration(2.0).sleep();
    }

    // open gripper
    g_hand.setStartStateToCurrentState();
    const auto grip_rtraj = getGripperTrajectory(g_hand, true);
    moveit::planning_interface::MoveGroupInterface::Plan p;
    p.trajectory_ = grip_rtraj;
    g_arm.execute(p);
  }
  return 1;
}
#endif // EXERCISE4_FUNCTIONS_H
