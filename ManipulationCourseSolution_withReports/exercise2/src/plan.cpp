/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 11/12/22
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */
#include <exercise2/functions.h>
#include <kdl_parser/kdl_parser.hpp>
#include <std_srvs/Empty.h>
#include <chrono>
#include <kdl/chainfksolverpos_recursive.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan");
    ros::NodeHandle node("~");

    std::string model_path;

    node.getParam("/URDFmodel", model_path);

    KDL::Tree lumi_tree;
    if (!kdl_parser::treeFromFile(model_path, lumi_tree)) {
        return false;
    }
    
    KDL::Chain lumi_chain;

    lumi_tree.getChain("base_link", "lumi_ee", lumi_chain);
    auto fk_solver = KDL::ChainFkSolverPos_recursive(lumi_chain);
    KDL::Frame next_pose;
    KDL::Frame prev_pose;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std_srvs::Empty srv_reset;
    ros::service::waitForService("/lumi_mujoco/reset");
    ros::service::call("/lumi_mujoco/reset", srv_reset);

    if (!turnOffPathSimplification()) {
        ROS_ERROR_STREAM("Cannot turn off simplification");
        return -1;
    }

    //Load MoveGroup interface and moveit visual tools
    moveit::planning_interface::MoveGroupInterface g_arm("lumi_arm");
    moveit_visual_tools::MoveItVisualTools vis("base_link");
    vis.loadMarkerPub(true);
    vis.deleteAllMarkers();
    vis.trigger();

    //Get Start state
    const auto state_ptr = g_arm.getCurrentState(10.0);
    if (!state_ptr) {
        ROS_ERROR_STREAM("Cannot get current state");
        return -1;
    }
    auto state = *state_ptr;

    const auto jmg = state.getJointModelGroup("lumi_arm"); //joint model group used for IK computation



    std::array<double, 7> jvalues = {1.15, -1.55, -1.68, -2.43, -0.14, 2.03, 0.68};
    std::array<std::string, 7> jnames = {"lumi_joint1", "lumi_joint2", "lumi_joint3", "lumi_joint4",
                                         "lumi_joint5", "lumi_joint6", "lumi_joint7"};

    auto start_state = state;
    auto goal_state = state;
    for (size_t i = 0; i < jnames.size(); ++i) {
        goal_state.setVariablePosition(jnames[i], jvalues[i]);
    }

    /*TODO: Set different planner, for available planners look at rosed lumi_moveit_config ompl_planning.yaml */
    //g_arm.setPlannerId("...")
    g_arm.setPlannerId("KPIECE");
    g_arm.setPlanningTime(5.0); //keep 5s for all planners

    g_arm.setStartState(start_state); //set start_state

    const auto start = std::chrono::steady_clock::now();

    auto plan = planToState(g_arm, goal_state); //compute plan form start to goal_state

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
    std::cout << "Computation time: " << dt.count() << " [ms]" << std::endl;

    vis.publishTrajectoryLine(plan, state.getLinkModel("lumi_ee"), jmg); //visualise plan
    vis.trigger();

    /*TODO: Compute some metrics on the path according to assignment. Check the documentation carefully for the hint.
      Hints: You can use forward kinematics (FK) to map from joint space -> cartesian space (end effector position).
        - plan.joint_trajectory.points //individual points in trajectory
        - plan.joint_trajectory.points[0].positions[0] //get the first joint value of the first point in path
        - The FK computation can be done using KDL as follow:
            - Define a JntArray: 
                KDL::JntArray joint_angles = KDL::JntArray(plan.joint_trajectory.points[j].positions.size());
                for (auto i = 0u; i < plan.joint_trajectory.points[j].positions.size(); i++) {
                        joint_angles(i) = plan.joint_trajectory.points[j].positions[i];
                    }
            - Compute FK (Input: joint_angles, Ouput: cartesian_pose):
                fk_solver.JntToCart(joint_angles, cartesian_pose);
        - Use the cartesian pose to calculate path length.
    */   
    g_arm.execute(trajectoryToPlan(plan));

    ros::Duration(1.0).sleep();


    return 0;
}
