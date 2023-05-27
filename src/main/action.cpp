/**
 * @file action.cpp
 * @author Wei Ling
 * @brief
 * @version 0.1
 * @date 2023-04-30
 *
 * @copyright Copyright (c) 2023
 * Receive action and send command to robot
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_visual_tools/moveit_visual_tools.h>    

#include "ccf/connection/MqttConnection.h"
#include "ccf/util/NodeUtil.h"
#include "motion_util.h"

std::string NODE_NAME = "first_demo";
std::string PLANNING_GROUP = "panda_arm";

const double tau = 2 * M_PI;

using CetiRosToolbox::getParameter;
using CetiRosToolbox::getPrivateParameter;

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n(NODE_NAME);

    ROS_INFO_NAMED("Robotic assembly demo", ">>>>>>>>>>>>>>>> WAITING FOR ROBOT INIT <<<<<<<<<<<<<<<");
    ros::WallDuration(3.0).sleep();
    ROS_INFO_NAMED("Robotic assembly demo", ">>>>>>>>>>>>>>>>> WAKING UP AFTER INIT <<<<<<<<<<<<<<<<");

    ros::spinOnce();
    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    group.setPlanningTime(45.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "First Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    Motions motion;
    motion.objectsPlacement(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    motion.pick(group);
    ros::WallDuration(1.0).sleep();

    motion.place(group);

    ros::waitForShutdown();
  return 0;
}

