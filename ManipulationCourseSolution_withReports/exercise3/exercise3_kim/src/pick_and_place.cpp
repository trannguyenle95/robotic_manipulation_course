/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 12/17/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Modified on 08/01/20
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#include <exercise3/functions.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Wait for the simulator to come online and then we reset it
    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset");
    ros::service::call("/lumi_mujoco/reset", srv_reset);

    // read poses from tf
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped pick_to_base = tfBuffer.lookupTransform("base_link", "pick", ros::Time(0), ros::Duration(2.0));
    geometry_msgs::TransformStamped place_to_base = tfBuffer.lookupTransform("base_link", "place", ros::Time(0), ros::Duration(2.0));
    /*std::cout << "-----Transforms-----" << std::endl;
    std::cout << "Pick:\n" << pick_to_base << std::endl;
    std::cout << "Place:\n" << place_to_base << std::endl << std::endl;*/

    // convert poses to Eigen
    Eigen::Isometry3d pick_eigen = tf2::transformToEigen(pick_to_base);
    Eigen::Isometry3d place_eigen = tf2::transformToEigen(place_to_base);
    /*std::cout << "-----Eigen poses-----" << std::endl;
    std::cout << "Pick:\n" << pick_eigen.matrix() << std::endl;
    std::cout << "Place:\n" << place_eigen.matrix() << std::endl << std::endl;*/


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
    robot_state::RobotState start_state = *state_ptr;

    const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(
        "lumi_arm"); // joint model group used for IK computation

    const std::string ee_link = "lumi_ee"; // Name of the end effector link
    const Eigen::Isometry3d arm_to_ee =
        state.getGlobalLinkTransform(g_arm.getEndEffectorLink()).inverse() *
        state.getGlobalLinkTransform(
        ee_link); // Transformation from base link to end effector link

    // compute required poses and visualize them

    Eigen::Isometry3d e1 = state.getGlobalLinkTransform(ee_link) *
             Eigen::Translation3d(0.1, 0.0, 0.0);
    Eigen::Isometry3d e2 = state.getGlobalLinkTransform(ee_link) *
             Eigen::Translation3d(-0.1, 0.0, 0.0) *
             Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

    Eigen::Isometry3d pre_grasp = pick_eigen * Eigen::Translation3d(0.0, 0.0, 0.1) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d grasp = pick_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d place_final = place_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

    //vis.publishAxis(e1);
    //vis.publishAxis(e2);
    vis.publishAxis(pre_grasp);
    vis.publishAxis(grasp);
    vis.publishAxis(place_final);
    vis.trigger();

    std::vector<moveit_msgs::RobotTrajectory> trajectories;

    // Planning from start to pre-grasp pose
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, pre_grasp, ee_link)) {
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
    
    // Planning from pre-grasp to grasp pose
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, grasp, ee_link)) {
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
    
    // Close gripper
    const moveit_msgs::RobotTrajectory close_traj = getGripperTrajectory(g_hand, false);
    trajectories.push_back(close_traj);
    
    // Planning from grasp to pre-grasp pose
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, pre_grasp, ee_link)) {
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
    
    // Planning from pre-grasp to place pose
    g_arm.setStartState(state);
    if (!state.setFromIK(jmg, place_final, ee_link)) {
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

    // cartesian pose
    state = start_state;
    g_arm.setStartState(state);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(pre_grasp * arm_to_ee.inverse(), pose);
    moveit_msgs::RobotTrajectory rtraj;
    const double d = g_arm.computeCartesianPath({pose}, 0.01, 1.4, rtraj);
    if (d < 0.99) {
        ROS_ERROR_STREAM("Cannot interpolate to the grasping position");
        return -1;
    }
    //trajectories.push_back(rtraj);

    // Visualise all trajectories
    for (const moveit_msgs::RobotTrajectory &t : trajectories) {
        vis.publishTrajectoryLine(t, state.getLinkModel(ee_link), jmg);
    }
    vis.trigger();

    if (askContinue()) {
        // Execute all trajectories
        for (const moveit_msgs::RobotTrajectory &t : trajectories)
            g_arm.execute(trajectoryToPlan(t));
    } else {
        // ...
    }

    // open gripper
    g_hand.setStartStateToCurrentState();
    // Information about getGripperTrajectory can be found in
    // exercise3/include/exercise3/functions.h
    const moveit_msgs::RobotTrajectory traj = getGripperTrajectory(g_hand, true);
    if (!traj.joint_trajectory.points.empty()) {
        g_hand.execute(trajectoryToPlan(traj));
    }

    return 0;
}
