import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os 
import xacro

packageName = 'eb65_description'
xacroRelativePath = 'model/eb65.urdf'
rvizRelativePath = 'rviz/eb65_display.rviz'

ros2controlRelativePath = 'config/eb65_controller.yaml'

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)
    ros2controlPath=os.path.join(pkgPath, ros2controlRelativePath)
    print(f"Loading Xacro from: {xacroModelPath}")

    robot_desc = xacro.process_file(xacroModelPath).toxml()
    robot_description = {'robot_description': robot_desc}

    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(name="gui", default_value="false", description="Start rviz")
    )
    gui = LaunchConfiguration("gui")

    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare(package="ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=launch.conditions.IfCondition(gui))
    
    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare(package="ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
        condition=launch.conditions.UnlessCondition(gui))
    
    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )   

    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "robot",
            "-allow_renaming",
            "true"])
    
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizConfigPath],
        condition=launch.conditions.IfCondition(gui)
    )

    control_node=launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2controlPath],
        output="both",
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner=launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Correct spawner for the robot controller
    robot_controller_spawner=launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", ros2controlPath],
        output="screen",
    )

    nodelist = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node
    ]

    return launch.LaunchDescription(declared_arguments + nodelist)