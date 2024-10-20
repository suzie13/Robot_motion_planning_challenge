import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    # Paths to configuration files
    urdf_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'ur5.urdf.xacro')
    srdf_file = os.path.join(get_package_share_directory('robot_motion_planning'), 'srdf', 'ur5.srdf')
    ompl_config = os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'ompl_planning.yaml')
    controllers_config = os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'controllers.yaml')
    moveit_config = os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'moveit_config.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'moveit.rviz')

    # Convert the URDF and SRDF to strings
    robot_description_config = xacro.process_file(urdf_file).toxml()
    with open(srdf_file, 'r') as srdf:
        robot_description_semantic_config = srdf.read()

    # Define the parameters to pass to nodes
    robot_description_param = {'robot_description': robot_description_config}
    robot_description_semantic_param = {'robot_description_semantic': robot_description_semantic_config}

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        ),

        # ROS 2 Control Node (for controller manager)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[robot_description_param, robot_description_semantic_param, controllers_config],
            output='screen',
            remappings=[
                ("~/robot_description", "robot_description"),
            ],
            arguments=['--ros-args', '--log-level', 'debug']
        ),

        # Add a delay to ensure controller_manager initializes before spawners
        TimerAction(
            period=5.0,
            actions=[
                # Spawner for joint_state_broadcaster
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_joint_state_broadcaster',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),

                # Spawner for position joint trajectory controller
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_pos_joint_traj_controller',
                    arguments=['pos_joint_traj_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                ),
            ],
        ),

        # Move Group (MoveIt2)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description_param,               # Pass the URDF
                robot_description_semantic_param,      # Pass the SRDF
                {
                    'ompl_planning_yaml': ompl_config,
                    'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                    'trajectory_execution.allowed_goal_duration_margin': 0.5,
                    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                    'moveit_config': moveit_config
                }
            ]
        ),

        # RViz for visualizing the planning and execution
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Custom motion planning node
        Node(
            package='robot_motion_planning',
            executable='robot_motion_node',
            output='screen',
            parameters=[robot_description_param, robot_description_semantic_param]  # Ensure parameters are passed here too
        )
    ])
