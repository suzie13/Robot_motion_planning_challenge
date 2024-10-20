#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_motion_node");

    // Create MoveGroup interface for the robot arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    // Create PlanningSceneInterface to add objects to the environment
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a PlanningSceneMonitor planning_scene_monitor::PlanningSceneMonitor
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");

    // Start the planning scene monitor to publish updates
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
    planning_scene_monitor->startSceneMonitor("/monitored_planning_scene");
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    // Define the box pose
    geometry_msgs::msg::PoseStamped box_pose;
    box_pose.header.frame_id = "base_link";
    box_pose.pose.position.x = 1.0;
    box_pose.pose.position.y = 0.0;
    box_pose.pose.position.z = 0.5;

    // Define the collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "box";
    collision_object.header.frame_id = "base_link";

    // Define the box as a primitive (size of the box: 0.5m x 0.5m x 0.5m)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.5, 0.5, 0.5};  // Length, width, height

    // Add the pose and primitive to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose.pose);
    collision_object.operation = collision_object.ADD;

    // Apply the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);

    // Set max velocity and acceleration scaling factors
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // Plan and move to the "home" position
    move_group.setNamedTarget("home");
    move_group.move();

    // Spin to keep the node alive (if there are callbacks in the future)
    rclcpp::spin_some(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
