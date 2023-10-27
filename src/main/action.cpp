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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::array<float,9> poses_pick = {0.5,0,0.53,-1.0,0.095,0.115,1.0,0.1,0.25};
    std::array<float,10> poses_default_place = {0,0.5,0.53,-1.0,0.1,0.25,-1.0,1.0,0.1,0.25};
    std::array<float,10> poses_place = {3,3,3,-1.0,0.1,0.25,-1.0,1.0,0.1,0.25};

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(group.getJointModelGroupNames().begin(),
              group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ","));
    std::cout<<"\n";

    Motions motion;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ObjectsPlacement>("ObjectsPlacement",std::ref(planning_scene_interface),std::ref(my_plan));
    factory.registerNodeType<PickAction>("PickAction",std::ref(group),std::ref(my_plan),poses_pick);
    factory.registerNodeType<PlaceAction>("PlaceAction",std::ref(group),std::ref(my_plan),poses_place);
    factory.registerNodeType<DefaultPlaceAction>("DefaultPlaceAction",std::ref(group),std::ref(my_plan),poses_default_place);

    auto tree = factory.createTreeFromFile("./src/project1/src/main/bt_tree.xml");
    tree.tickWhileRunning();

    ros::WallDuration(2.0).sleep();
    ros::shutdown();
  return 0;
}

