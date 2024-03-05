import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    gazebo_pkg = "gazebo_ros"
    spawn_node = "spawn_entity.py"
    spawn_entity = Node(
        package=gazebo_pkg,
        executable=spawn_node,
        name="spawn_entity_node",
        arguments=["-topic", "robot_description", "-entity", "my_bot", "-x", "0", "-y", "0"],
        output="screen"
    )
    
    pkg_name = "diff_bot"
    rsp_file = "rsp.launch.py"
    rsp_path = os.path.join(get_package_share_directory(pkg_name), "launch", rsp_file)
    
    rsp = IncludeLaunchDescription([rsp_path])
    
    
    gazebo_file = "gazebo.launch.py"
    gazebo_path = os.path.join(get_package_share_directory(gazebo_pkg), "launch", gazebo_file)
    world_path = os.path.join(get_package_share_directory(pkg_name), "worlds", "sim_world.world")
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_path]),
        launch_arguments=[('world', world_path)]
        )
    
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), "rviz", "show_robot.rviz")
    use_rviz = LaunchConfiguration("rviz", default=False)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(use_rviz)
        )
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz
    ])
    
    