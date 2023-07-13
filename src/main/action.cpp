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
#include "util/motion_util.h"

std::string NODE_NAME = "first_demo";
std::string PLANNING_GROUP = "panda_arm";

const double tau = 2 * M_PI;

// using CetiRosToolbox::getParameter;
// using CetiRosToolbox::getPrivateParameter;

using namespace BT;

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n(NODE_NAME);

    ROS_INFO_NAMED("Robotic assembly demo", ">>>>>>>>>>>>>>>> WAITING FOR ROBOT INIT <<<<<<<<<<<<<<<");
    //ros::WallDuration(3.0).sleep();
    ROS_INFO_NAMED("Robotic assembly demo", ">>>>>>>>>>>>>>>>> WAKING UP AFTER INIT <<<<<<<<<<<<<<<<");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    //ros::WallDuration(1.0).sleep();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
    group.setPlanningTime(45.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(group.getJointModelGroupNames().begin(),
              group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    Motions motion;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ObjectsPlacement>("ObjectsPlacement", planning_scene_interface);
    factory.registerNodeType<PickAction>("PickAction", &group);
    factory.registerNodeType<PlaceAction>("PlaceAction", &group);

    auto tree = factory.createTreeFromFile("./../bt_tree.xml");
    tree.tickWhileRunning();
    

    

    //motion.objectsPlacement(planning_scene_interface);

    //visual_tools.deleteAllMarkers();
    //visual_tools.publishText(text_pose, "Objects pose", rvt::WHITE, rvt::XLARGE);
    //visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    //motion.pick(group);

    //visual_tools.deleteAllMarkers();
    //visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    //motion.place(group);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (place objects) %s", success ? "" : "FAILED");
    //visual_tools.deleteAllMarkers();
    //visual_tools.trigger();

    ros::WallDuration(2.0).sleep();
    ros::shutdown();
  return 0;
}

