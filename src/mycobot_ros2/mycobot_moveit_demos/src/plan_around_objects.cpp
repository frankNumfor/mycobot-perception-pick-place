/**
 * @file plan_around_objects.cpp
 * @brief Demonstrates basic usage of MoveIt with ROS 2 and collision avoidance.
 *
 * This program initializes a ROS 2 node, sets up a MoveIt interface for a robot manipulator,
 * creates a collision object in the scene, plans a motion to a target pose while avoiding
 * the collision object, and executes the planned motion.
 *
 * @author Addison Sears-Collins
 * @date December 15, 2024
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "plan_around_objects",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("plan_around_objects");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(1.0);
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());

  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              arm_group_interface.getRobotModel() };

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;
        return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools, jmg = arm_group_interface.getRobotModel()->getJointModelGroup("arm")](
	  auto const trajectory) {moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);};

  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.128;
    msg.pose.position.y = -0.266;
    msg.pose.position.z = 0.111;
    msg.pose.orientation.x = 0.635;
    msg.pose.orientation.y = -0.268;
    msg.pose.orientation.z = 0.694;
    msg.pose.orientation.w = 0.206;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);

  auto const collision_object = [frame_id = arm_group_interface.getPlanningFrame(), &node, &logger] {

    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.header.stamp = node->now();
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.50;

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.22;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    RCLCPP_INFO(logger, "Created collision object: %s", collision_object.id.c_str());
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f",
      primitive.dimensions[primitive.BOX_X],
      primitive.dimensions[primitive.BOX_Y],
      primitive.dimensions[primitive.BOX_Z]);
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose.position.x, box_pose.position.y, box_pose.position.z);

    return collision_object;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();

  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    arm_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}