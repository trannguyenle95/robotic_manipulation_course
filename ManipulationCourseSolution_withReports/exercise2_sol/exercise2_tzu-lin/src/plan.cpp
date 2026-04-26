/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2020/01/23
 *     Author: Tzu Lin, Huang <tzu-lin.huang@aalto.fi>
 */
#include <chrono>
#include <exercise2/functions.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "plan");
  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std_srvs::Empty srv_reset;
  ros::service::waitForService("/lumi_mujoco/reset");
  ros::service::call("/lumi_mujoco/reset", srv_reset);

  if (!turnOffPathSimplification()) {
    ROS_ERROR_STREAM("Cannot turn off simplification");
    return -1;
  }

  // Load MoveGroup interface and moveit visual tools
  moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
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

  const auto jmg = state.getJointModelGroup("lumi_arm"); // joint model group used for IK computation




  std::array<double, 7> jvalues = {1.15,  -1.55, -1.68, -2.43, -0.14, 2.03,  0.68};
  std::array<std::string, 7> jnames = {"lumi_joint1", "lumi_joint2", "lumi_joint3", "lumi_joint4",
      "lumi_joint5", "lumi_joint6", "lumi_joint7"};

  auto start_state = state;
  auto goal_state = state;
  for (size_t i = 0; i < jnames.size(); ++i) {
    goal_state.setVariablePosition(jnames[i], jvalues[i]);
  }

  // todo set different planner, for available planners look at rosed
  // lumi_moveit_config ompl_planning.yaml
  //    g_arm.setPlannerId("...")
  for (int index = 0; index < 5; ++index) {
    g_arm.setPlannerId("PRM");
    g_arm.setPlanningTime(5.0); // keep 5s for all planners

    g_arm.setStartState(start_state); // set start_state

    const auto start = std::chrono::steady_clock::now();

    auto plan =
        planToState(g_arm, goal_state); // compute plan form start to goal_state

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);
    std::cout << "Computation time: " << dt.count() << " [ms]" << std::endl;

    vis.publishTrajectoryLine(plan, state.getLinkModel("lumi_ee"),
                              jmg); // visualise plan
    vis.trigger();

    g_arm.execute(trajectoryToPlan(plan));
    // todo compute some metrics on the path according to assignment
    //    plan.joint_trajectory.points //individual points in trajectory
    //    plan.joint_trajectory.points[0].positions[0] //get the first joint
    //    value of the first point in path

    //    LOG
    ROS_INFO("Reference frame: %s", g_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffector frame: %s", g_arm.getEndEffectorLink().c_str());

    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(
        new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    int num_waypoints =
        plan.joint_trajectory.points
            .size(); // gets the number of waypoints in the trajectory
    std::cout << "NUM OF WAYPOINTS :" << num_waypoints << std::endl;

    const std::vector<std::string> joint_names =
        plan.joint_trajectory.joint_names; // gets the names of the joints being
                                           // updated in the trajectory
    kinematic_state->setVariablePositions(
        joint_names, plan.joint_trajectory.points.at(0).positions);

    std::string end_effector = "lumi_ee";
    Eigen::Affine3d current_end_effector_state =
        kinematic_state->getGlobalLinkTransform(end_effector);

    Eigen::Affine3d next_end_effector_state;
    double euclidean_distance;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint,
        *next_waypoint;

    double result = 0.0;

    for (int i = 0; i < num_waypoints - 1; i++) // loop through all waypoints
    {
      curr_waypoint = &plan.joint_trajectory.points.at(i);
      next_waypoint = &plan.joint_trajectory.points.at(i + 1);

      // set joints for next waypoint
      kinematic_state->setVariablePositions(joint_names,
                                            next_waypoint->positions);

      // do forward kinematics to get cartesian positions of end effector for
      // next waypoint
      next_end_effector_state =
          kinematic_state->getGlobalLinkTransform(end_effector);

      // get euclidean distance between the two waypoints
      euclidean_distance =
          pow(pow(next_end_effector_state.translation()[0] -
                      current_end_effector_state.translation()[0],
                  2) +
                  pow(next_end_effector_state.translation()[1] -
                          current_end_effector_state.translation()[1],
                      2) +
                  pow(next_end_effector_state.translation()[2] -
                          current_end_effector_state.translation()[2],
                      2),
              0.5);

      // std::cout << "Euclidean Distance: " << euclidean_distance << std::endl;
      result += euclidean_distance;

      // update current_end_effector_state for next iteration
      current_end_effector_state = next_end_effector_state;
    }

    std::cout << "Trajectory Path Length (Euclidean Distance) " << result
              << std::endl;
    ros::service::call("/lumi_mujoco/reset", srv_reset);
  }

  ros::Duration(1.0).sleep();
  return 0;
}
