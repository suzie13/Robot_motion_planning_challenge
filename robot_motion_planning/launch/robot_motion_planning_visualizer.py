import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Resolve the path to the robot_description package
    robot_description_package = get_package_share_directory('robot_description')
    urdf_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'ur5.urdf.xacro')
    srdf_file = os.path.join(get_package_share_directory('robot_motion_planning'), 'srdf', 'ur5.srdf')

    rviz_config_file = os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'moveit.rviz')

    # robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    # Declare arguments for parameters
    declare_joint_limits_file_arg = DeclareLaunchArgument(
        'joint_limits_parameters_file',
        default_value=os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'joint_limits.yaml'),
        description='Path to the joint limits parameters file'
    )

    declare_kinematics_file_arg = DeclareLaunchArgument(
        'kinematics_parameters_file',
        default_value=os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'default_kinematics.yaml'),
        description='Path to the kinematics parameters file'
    )

    declare_physical_file_arg = DeclareLaunchArgument(
        'physical_parameters_file',
        default_value=os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'physical_parameters.yaml'),
        description='Path to the kinematics parameters file'
    )

    declare_visual_file_arg = DeclareLaunchArgument(
        'visual_parameters_file',
        default_value=os.path.join(get_package_share_directory('robot_motion_planning'), 'config', 'visual_parameters.yaml'),
        description='Path to the kinematics parameters file'
    )

    # Define robot description using xacro command with arguments for parameters

    robot_description_content = Command([
        'xacro ', urdf_file,
        # ' mesh_path:=', robot_description_package,
        ' joint_limits_parameters_file:=', LaunchConfiguration('joint_limits_parameters_file'),
        ' kinematics_parameters_file:=', LaunchConfiguration('kinematics_parameters_file'),
        ' physical_parameters_file:=', LaunchConfiguration('physical_parameters_file'),
        ' visual_parameters_file:=', LaunchConfiguration('visual_parameters_file')
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # robot_description_semantic = {'robot_description_semantic': Command(['cat ', srdf_file])}

    robot_description_semantic_content = Command(['cat ', srdf_file])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }


    return LaunchDescription([

        # Add the declared arguments to LaunchDescription
        declare_joint_limits_file_arg,
        declare_kinematics_file_arg,
        declare_physical_file_arg,
        declare_visual_file_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[robot_description, robot_description_semantic]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
        package='robot_motion_planning',
        executable='robot_motion_node',
        output='screen',
        parameters=[robot_description, robot_description_semantic]
        )
    ])
