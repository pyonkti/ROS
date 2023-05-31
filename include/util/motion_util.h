#ifndef __MOTION_H__
#define __MOTION_H__

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Motions{
    private:


    public:
        Motions(/* args */);
        ~Motions();
        void openUpGripper(trajectory_msgs::JointTrajectory& posture);
        void closeGripper(trajectory_msgs::JointTrajectory& posture);
        void pick(moveit::planning_interface::MoveGroupInterface& move_group);
        void place(moveit::planning_interface::MoveGroupInterface& move_group);
        void objectsPlacement(moveit::planning_interface::PlanningSceneInterface& planning_scene);
};

Motions::Motions(){
}
Motions::~Motions()
{
}
#endif // __MOTION_UTIL_H__