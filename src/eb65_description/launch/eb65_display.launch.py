from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('eb65_description'), 'urdf', 'eb65.urdf')
    rviz_path = os.path.join(get_package_share_path('eb65_description'), 'rviz', 'eb65_display.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])