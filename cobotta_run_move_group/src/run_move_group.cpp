#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm_controller";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.5;

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  // Start Demo
  // Motion 0_1
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::PoseStamped cartesian_path_z = move_group.getCurrentPose();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(cartesian_path_z.pose);

  cartesian_path_z.header.frame_id = "base_link";
  cartesian_path_z.pose.position.z += 0.15;
  move_group.setPoseTarget(cartesian_path_z, "link_J6_1");
  
  waypoints.push_back(cartesian_path_z.pose);
  move_group.clearPathConstraints();

  // Motion 0_2
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::PoseStamped cartesian_path_x = move_group.getCurrentPose();
  cartesian_path_x.header.frame_id = "base_link";
  cartesian_path_x.pose.position.x -= 0.1;
  move_group.setPoseTarget(cartesian_path_x, "link_J6_1");
  move_group.move();
  move_group.clearPathConstraints();

  // Motion 1
  std::vector<double> joint_group_positions;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[1] = -0.6;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();

  // Motion 2
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.24;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.12;
  move_group.setPoseTarget(another_pose);
  move_group.move();
  move_group.clearPathConstraints();

  // Motion 3
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = -0.03;
  pose1.position.y = -0.16;
  pose1.position.z = 0.1;
  move_group.setPoseTarget(pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();

  // Motion 4 CollisionObject
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.04;
  primitive.dimensions[primitive.BOX_Y] = 1.0;
  primitive.dimensions[primitive.BOX_Z] = 0.09;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.12;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.045;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects); //新しい干渉検出器を追加

  move_group.setStartState(*move_group.getCurrentState());
  // move_group.setPositionTarget(0.22, 0.0, 0.13, "link_J6_1");
  move_group.setPoseTarget(another_pose, "link_J6_1");
  move_group.setPlanningTime(20.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan %s", success ? "" : "FAILED");
  move_group.move();
  move_group.clearPathConstraints();

  // Motion 5 CylinderObject
  moveit_msgs::msg::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";
  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.05;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.015;

  object_to_attach.header.frame_id = move_group.getEndEffectorLink();
  geometry_msgs::msg::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = -0.065;

  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  RCLCPP_INFO(LOGGER, "Attach the object to the robot");
  std::vector<std::string> touch_links;
  touch_links.push_back("left_gripper_1");
  touch_links.push_back("right_gripper_1");
  move_group.attachObject(object_to_attach.id, "gripper_controller", touch_links);

  move_group.setStartStateToCurrentState();
  geometry_msgs::msg::Pose pose2;
  pose2.orientation.w = 1.0;
  pose2.position.x = 0.24;
  pose2.position.y = -0.15;
  pose2.position.z = 0.15;
  move_group.setPoseTarget(pose2);
  move_group.setPlanningTime(20.0);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan %s", success ? "" : "FAILED");
  move_group.move();
  move_group.clearPathConstraints();

  // Delete objects
  RCLCPP_INFO(LOGGER, "Detach the object from the robot");
  move_group.detachObject(object_to_attach.id);

  RCLCPP_INFO(LOGGER, "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  rclcpp::shutdown();
  return 0;
}
