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
        std::vector<moveit_msgs::Grasp> pick(std::array<float,9> grasp_poses);
        std::vector<moveit_msgs::PlaceLocation> place(std::array<float,10> grasp_poses);
        std::vector<moveit_msgs::CollisionObject> objectsPlacement();
};

class PickAction: public SyncActionNode{
    private:
        moveit::planning_interface::MoveGroupInterface& move_group;
        moveit::planning_interface::MoveGroupInterface::Plan& my_plan;
        std::array<float,9> grasp_poses_pick;
    public:
        PickAction(const std::string& name, const NodeConfig& config, 
            moveit::planning_interface::MoveGroupInterface& arg_move_group,
            moveit::planning_interface::MoveGroupInterface::Plan& arg_plan,
            std::array<float,9> arg_poses
            ):
            SyncActionNode(name, config),
            move_group(arg_move_group),
            my_plan(arg_plan),
            grasp_poses_pick(arg_poses){}
        NodeStatus tick() override;
        static PortsList providedPorts(){
            return {};
        }
};

class PlaceAction: public SyncActionNode{
    private:
        moveit::planning_interface::MoveGroupInterface& move_group;
        moveit::planning_interface::MoveGroupInterface::Plan& my_plan;
        std::array<float,10> grasp_poses_place;
    public:
        PlaceAction(const std::string& name, const NodeConfig& config, 
            moveit::planning_interface::MoveGroupInterface& arg_move_group,
            moveit::planning_interface::MoveGroupInterface::Plan& arg_plan,
            std::array<float,10> arg_poses):
            SyncActionNode(name, config),
            move_group(arg_move_group),
            my_plan(arg_plan),
            grasp_poses_place(arg_poses){}
        NodeStatus tick() override;
        static PortsList providedPorts(){
            return {};
        }
};

class ObjectsPlacement: public SyncActionNode{
    private:
        moveit::planning_interface::PlanningSceneInterface& planning_scene;
        moveit::planning_interface::MoveGroupInterface::Plan& my_plan;
    public:
        ObjectsPlacement(const std::string& name, const NodeConfig& config, 
            moveit::planning_interface::PlanningSceneInterface& arg_planning_scene,
            moveit::planning_interface::MoveGroupInterface::Plan& arg_plan):
            SyncActionNode(name, config),
            planning_scene(arg_planning_scene),
            my_plan(arg_plan){}    
        NodeStatus tick() override;
        static PortsList providedPorts(){
            return {};
        }
};

#endif // __MOTION_UTIL_H__