/*
 *    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
 *    Copyright (C) 2024 Usama Ali
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License v3 as published by
 *    the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License v3 for more details.
 *
 *    You should have received a copy of the GNU General Public License v3
 *    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *    Authors: Usama Ali <usama.ali@thws.de>
 *             Adrian Mueller <adrian.mueller@thws.de>
 *             Lan Wu <Lan.Wu-2@uts.edu.au>
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_ur5e");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const double tau = 2 * M_PI;

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;

    geometry_msgs::Pose target_pose2;
    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.28;
    target_pose2.position.y = -0.2;
    target_pose2.position.z = 0.5;

    std::vector<double> joint_group_positions;
    // std::vector<double> joint_group_positions1 = {-1.2495972614104018, -1.3419874498678586, 1.813313415013816, -1.9535953734573601, -1.812795644521525, 0.0};
    std::vector<double> joint_group_positions1 = {-1.03839740169054, -0.12319996255593763, 0.2199114857512856, -1.9540706305328515, -1.8133272796520288, 0.0};
    // std::vector<double> joint_group_positions2 = {-0.7215984110590741, -1.1087975324745665, 1.8126989611213107, -2.270394630404935, -1.5663962520418677, 0.0};
    std::vector<double> joint_group_positions2 = {-2.692793817212441, -0.12319996255593763, 0.2199114857512856, -1.9540706305328515, -1.8133272796520288, 0.0};
    std::vector<double> joint_group_positions3 = {0.6159987475152521, -1.3419874498678586, 1.813313415013816, -1.9535953734573601, -1.812795644521525, 0.0};
    std::vector<double> joint_group_positions4 = {0.15839949531573294, -1.1087975324745665, 1.8126989611213107, -2.270394630404935, -1.5663962520418677, 0.0};
    std::vector<double> joint_group_positions5 = {0.9679978035387312, -1.0031978689307446, 2.173594850051062, -2.270394630404935, -1.5663962520418677, 0.0};

    //-1.03839740169054, -0.12319996255593763, 0.2199114857512856, -1.9540706305328515, -1.8133272796520288, 0.0
    //-2.692793817212441, -0.12319996255593763, 0.2199114857512856, -1.9540706305328515, -1.8133272796520288, 0.0


    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    int input = -1;
    std::cout << "Plan to Pose\n[1]: Pose 1 (links oben)\n[2]: Pose 2 (links unten)\n[3]: Pose 3 (rechts oben)\n[4]: Pose 4 (rechts unten)\n[5]: Pose 5 (rechts unten)\n";

    while (ros::ok()&&input!=0)
    {
        std::cin >> input;

        switch (input)
        {
        case 1:
            // move_group_interface.setPoseTarget(target_pose1);
            move_group_interface.setJointValueTarget(joint_group_positions1);
            break;
        case 2:
            // move_group_interface.setPoseTarget(target_pose2);
            move_group_interface.setJointValueTarget(joint_group_positions2);
            break;
        case 3:
            // move_group_interface.setPoseTarget(target_pose2);
            move_group_interface.setJointValueTarget(joint_group_positions3);
            break;
        case 4:
            // move_group_interface.setPoseTarget(target_pose2);
            move_group_interface.setJointValueTarget(joint_group_positions4);
            break;
        case 5:
            // move_group_interface.setPoseTarget(target_pose2);
            move_group_interface.setJointValueTarget(joint_group_positions5);
            break;
        
        default:
            break;
        }

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // joint_group_positions[0] = -tau / 6;
        // move_group_interface.setJointValueTarget(joint_group_positions);

        move_group_interface.setMaxVelocityScalingFactor(0.05);
        move_group_interface.setMaxAccelerationScalingFactor(0.05);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        // ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        if(success)
        {
            // move_group_interface.move();
            move_group_interface.execute(my_plan);
        }
    }
}


 

