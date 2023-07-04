#ifndef __MOTION_H__
#define __MOTION_H__

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

class Motions{
    private:

    public:
        ~Motions(){};
        static void openUpGripper(trajectory_msgs::JointTrajectory& posture);
        static void closeGripper(trajectory_msgs::JointTrajectory& posture);
};

class PickAction: public SyncActionNode{
    private:
        moveit::planning_interface::MoveGroupInterface& move_group;
    public:
        PickAction(const std::string& name, const NodeConfig& config, moveit::planning_interface::MoveGroupInterface& move_group):
            SyncActionNode(name, config),
            move_group(move_group){}
        NodeStatus tick() override;
};

class PlaceAction: public SyncActionNode{
    private:
        moveit::planning_interface::MoveGroupInterface& move_group;
    public:
        PlaceAction(const std::string& name, const NodeConfig& config, moveit::planning_interface::MoveGroupInterface& move_group):
            SyncActionNode(name, config),
            move_group(move_group){}
        NodeStatus tick() override;
};

class ObjectsPlacement: public SyncActionNode{
    private:
        moveit::planning_interface::PlanningSceneInterface& planning_scene;
    public:
        ObjectsPlacement(const std::string& name, const NodeConfig& config, moveit::planning_interface::PlanningSceneInterface& planning_scene):
            SyncActionNode(name, config),
            planning_scene(planning_scene){}
        NodeStatus tick() override;
};

#endif // __MOTION_UTIL_H__