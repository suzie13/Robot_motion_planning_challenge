from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Generate robot_description from URDF/Xacro
    urdf_file = '/home/sushma/project_1.0/robot_arm/ur5_robot.urdf'
    robot_description_content = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content
            },
            '/home/sushma/project_1.0/robot_arm/install/robot_motion_planning/share/robot_motion_planning/config/controllers.yaml']
        )
    ])
