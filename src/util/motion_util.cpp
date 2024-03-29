#include "util/motion_util.h"

const double tau = 2 * M_PI;
using namespace BT;

void Motions::openUpGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void Motions::closeGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

std::vector<moveit_msgs::Grasp> Motions::pick(std::array<float,9> grasp_poses){
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(tau / 2, 0, - tau / 8);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = grasp_poses[0];
  grasps[0].grasp_pose.pose.position.y = grasp_poses[1];
  grasps[0].grasp_pose.pose.position.z = grasp_poses[2];

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.z = grasp_poses[3];
  grasps[0].pre_grasp_approach.min_distance = grasp_poses[4];
  grasps[0].pre_grasp_approach.desired_distance = grasp_poses[5];

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = grasp_poses[6];
  grasps[0].post_grasp_retreat.min_distance = grasp_poses[7];
  grasps[0].post_grasp_retreat.desired_distance = grasp_poses[8];

  openUpGripper(grasps[0].pre_grasp_posture);
  closeGripper(grasps[0].grasp_posture);

  // return type changed, no longer implemented here
  //move_group.setSupportSurfaceName("table1");
  //move_group.pick("object", grasps);
  return grasps;
}

NodeStatus PickAction::tick(){
  Motions motion;
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = grasp_poses_pick[0];
  target_pose1.position.y = grasp_poses_pick[1];
  target_pose1.position.z = grasp_poses_pick[2];
  target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose1);
  if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    ROS_INFO_NAMED("tutorial", "Visualizing plan (pick object)");
      move_group.setSupportSurfaceName("table1");
      move_group.pick("object", motion.pick(grasp_poses_pick));
    return NodeStatus::SUCCESS;
  }else {
    ROS_INFO_NAMED("tutorial", "Pick Plan failed, MoveItErrorCode: %d",move_group.plan(my_plan).val);
    return NodeStatus::FAILURE;};
}

std::vector<moveit_msgs::PlaceLocation> Motions::place(std::array<float,10> grasp_poses){
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau/4);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = grasp_poses[0];
  place_location[0].place_pose.pose.position.y = grasp_poses[1];
  place_location[0].place_pose.pose.position.z = grasp_poses[2];

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = grasp_poses[3];
  place_location[0].pre_place_approach.min_distance = grasp_poses[4];
  place_location[0].pre_place_approach.desired_distance = grasp_poses[5];

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y = grasp_poses[6];
  place_location[0].post_place_retreat.direction.vector.z = grasp_poses[7];
  place_location[0].post_place_retreat.min_distance = grasp_poses[8];
  place_location[0].post_place_retreat.desired_distance = grasp_poses[9];

  openUpGripper(place_location[0].post_place_posture);

  // return type changed, no longer implemented here
  //move_group.setSupportSurfaceName("table2");
  //move_group.place("object", place_location);
  return place_location;
}

NodeStatus PlaceAction::tick(){
  Motions motion;
  grasp_poses_place[6] = 0;
  geometry_msgs::Pose target_pose1 = move_group.getRandomPose().pose;
  ROS_INFO_NAMED("tutorial", "%f,%f,%f",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
  grasp_poses_place[0] = target_pose1.position.x;
  grasp_poses_place[1] = target_pose1.position.y;
  grasp_poses_place[2] = target_pose1.position.z;
  target_pose1.position.z += 0.25;
  target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  move_group.setStartStateToCurrentState();
  move_group.setApproximateJointValueTarget(target_pose1);
  if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {   
    move_group.setSupportSurfaceName("table2");
    if (move_group.place("object", motion.place(grasp_poses_place)) == moveit::core::MoveItErrorCode::SUCCESS){
        ROS_INFO_NAMED("tutorial", "Visualizing plan (assigned place object)");
        return NodeStatus::SUCCESS;
    }else{
        ROS_INFO_NAMED("tutorial", "Place failed");
        return NodeStatus::FAILURE;
    }    
  }else{
    ROS_INFO_NAMED("tutorial", "Place Plan failed, MoveItErrorCode: %d",move_group.plan(my_plan).val);
    return NodeStatus::FAILURE;
  }
}

NodeStatus DefaultPlaceAction::tick(){
  Motions motion;
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = grasp_poses_place[0];
  target_pose1.position.y = grasp_poses_place[1];
  target_pose1.position.z = (grasp_poses_place[2]+ 0.25);
  target_pose1.orientation.x = move_group.getCurrentPose().pose.orientation.x;
  target_pose1.orientation.y = move_group.getCurrentPose().pose.orientation.y;
  target_pose1.orientation.z = move_group.getCurrentPose().pose.orientation.z;
  target_pose1.orientation.w = move_group.getCurrentPose().pose.orientation.w;
  move_group.setStartStateToCurrentState();
  move_group.setApproximateJointValueTarget(target_pose1);
  if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    ROS_INFO_NAMED("tutorial", "Visualizing plan (default place object)");
    move_group.setSupportSurfaceName("table2");
    move_group.place("object", motion.place(grasp_poses_place));
    return NodeStatus::SUCCESS;
  }else{
    ROS_INFO_NAMED("tutorial", "Default Place Plan failed, MoveItErrorCode: %d",move_group.plan(my_plan).val);
    return NodeStatus::FAILURE;
  }
}

std::vector<moveit_msgs::CollisionObject> Motions::objectsPlacement(){
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  collision_objects[1].primitives.resize(1);

  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.03;
  collision_objects[2].primitives[0].dimensions[1] = 0.03;
  collision_objects[2].primitives[0].dimensions[2] = 0.03;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.415;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  // return type changed, no longer implemented here
  //planning_scene.applyCollisionObjects(collision_objects);
  return collision_objects;
}

NodeStatus ObjectsPlacement::tick(){
  Motions motion;
  planning_scene.applyCollisionObjects(motion.objectsPlacement());
  return NodeStatus::SUCCESS;
}
