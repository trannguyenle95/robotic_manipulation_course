/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 1/21/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
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

    auto planner_names = {"PRM", "RRT", "RRTstar", "KPIECE"};
    char flag = '\0';

    auto start_state = state;
    auto goal_state = state;
    for (size_t i = 0; i < jnames.size(); ++i) {
        goal_state.setVariablePosition(jnames[i], jvalues[i]);
    }
    std::cout << "Starting position: [  ";
    for (auto i = 0u; i < 7; i++) {
        std::cout << start_state.getVariablePosition(i) << "  ";
    }
    std::cout << "]" << std::endl;

    for (auto p : planner_names) {
        if (flag == 's') break;
        std::cout << "\nUsing " << p << " planner\n" << std::endl;
        g_arm.setPlannerId(p);
        g_arm.setPlanningTime(5.0);
        
        for (auto i = 0u; i < 5; i++) {
            ros::service::call("/lumi_mujoco/reset", srv_reset);
            vis.deleteAllMarkers();
            vis.trigger();
            std::cout << "Calculating plan #" << i+1 << "..." << std::endl;
            const auto start = std::chrono::steady_clock::now();
            
            auto plan = planToState(g_arm, goal_state);
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
            std::cout << "Computation time for " << p << " plan #" << i+1 << ": " << dt.count() << " [ms]" << std::endl;
            vis.publishTrajectoryLine(plan, state.getLinkModel("lumi_ee"), jmg);
            vis.trigger();
            double path_length = 0.0f;
            
            for (auto j = 1u; j < plan.joint_trajectory.points.size(); j++) {
                KDL::JntArray next_angles = KDL::JntArray(plan.joint_trajectory.points[j].positions.size());
                KDL::JntArray prev_angles = KDL::JntArray(plan.joint_trajectory.points[j-1].positions.size());
                
                for (auto i = 0u; i < plan.joint_trajectory.points[j].positions.size(); i++) {
                    next_angles(i) = plan.joint_trajectory.points[j].positions[i];
                    prev_angles(i) = plan.joint_trajectory.points[j-1].positions[i];
                }
                
                fk_solver.JntToCart(next_angles, next_pose);
                fk_solver.JntToCart(prev_angles, prev_pose);
                auto xdiff = next_pose.p.x() - prev_pose.p.x();
                auto ydiff = next_pose.p.y() - prev_pose.p.y();
                auto zdiff = next_pose.p.z() - prev_pose.p.z();
                path_length += sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
            }
            std::cout << "Path length (in Cartesian space): " << path_length << std::endl;
            g_arm.execute(trajectoryToPlan(plan));
            ros::Duration(0.25).sleep();
            std::cout << "Press ENTER to continue or type s to stop." << std::endl;
            flag = getchar();
            if (flag == 's') break;
        }
    }

    //todo compute some metrics on the path according to assignment
//    plan.joint_trajectory.points //individual points in trajectory
//    plan.joint_trajectory.points[0].positions[0] //get the first joint value of the first point in path


    ros::Duration(1.0).sleep();
    if (flag != 's') std::cout << "All sequences completed" << std::endl;
    else std::cout << "Sequence aborted" << std::endl;


    return 0;
}
