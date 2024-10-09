#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_motion_node");

    // Create MoveGroup interface for your robot arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");


    // Planning
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    
    // Example motion to home position
    move_group.setNamedTarget("home");
    move_group.move();

    rclcpp::shutdown();
    return 0;
}
