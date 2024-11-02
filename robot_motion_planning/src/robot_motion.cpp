#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class RobotMotionNode : public rclcpp::Node
{
public:
    RobotMotionNode(const std::shared_ptr<rclcpp::Node>& node) : Node("robot_motion_node"), node_(node)
    {
        // Create MoveGroup interface for the robot arm using shared node pointer
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "manipulator");

        // Create PlanningSceneInterface to add objects to the environment
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Initialize PlanningSceneMonitor
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");

        planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        // Add a collision box to the environment
        addCollisionBox();

        RCLCPP_INFO(this->get_logger(), "Subscribing to /joint_states");

        // Subscribe to joint states using shared node pointer
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&RobotMotionNode::jointStateCallback, this, std::placeholders::_1));

        // Publisher to send joint commands to hold position
        joint_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/pos_joint_traj_controller/joint_trajectory", 10);

        // Initialize the robot to the "home" position
        move_group_->setNamedTarget("home");
        move_group_->move();

        // Set max velocity and acceleration scaling factors
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Print the time the joint state is received
        RCLCPP_INFO(this->get_logger(), "Joint state received with %zu joints at time: %ld",
            msg->name.size(), this->now().nanoseconds());

        // Publish a trajectory to hold the robot's current joint positions
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = msg->name;  // Copy joint names

        // Create a point to hold joint positions
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = msg->position;  // Maintain current joint positions
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);  // Set time for control loop

        // Add the point to the trajectory message
        traj_msg.points.push_back(point);

        // Publish the trajectory to hold the robot in place
        joint_cmd_pub_->publish(traj_msg);
    }

    void addCollisionBox()
    {
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
        planning_scene_interface_->applyCollisionObject(collision_object);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub_;
};

int main(int argc, char** argv)
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMotionNode>(rclcpp::Node::make_shared("robot_motion_node"));

    // Keep the node running
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
