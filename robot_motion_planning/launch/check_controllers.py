from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Generate robot_description from URDF file
    urdf_file = '/home/sushma/project_1.0/robot_arm/ur5_robot.urdf'
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    robot_description_param = {'robot_description': robot_description_content}

    controller_config = get_package_share_directory('robot_motion_planning') + '/config/controllers.yaml'

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param],  # Continue using robot_description as parameter
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[robot_description_param],  # Use robot_description as parameter
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[robot_description_param, controller_config],  # Pass robot_description as parameter
            remappings=[
                ("~/robot_description", "robot_description"),
            ],
            arguments=['--ros-args', '--log-level', 'controller_manager:=debug'],
        ),

        # Add a delay to ensure controller_manager initializes before spawners
        TimerAction(
            period=5.0,  # 5-second delay before spawning controllers
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
    ])
