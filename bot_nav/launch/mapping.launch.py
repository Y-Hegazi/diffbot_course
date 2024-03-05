from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition




def generate_launch_description():
    
    package_name = "bot_nav"
    config_dir = os.path.join(get_package_share_directory(package_name), "config")
    mapping_config_file = 'cartographer.lua'
    rviz_config_file = os.path.join(get_package_share_directory(package_name), "rviz", "mapping.rviz")
    use_rviz = LaunchConfiguration("rviz", default=True)

    cartographer = Node(

        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=["-configuration_directory", config_dir,
                   "-configuration_basename", mapping_config_file]
    )

    grid = Node(
        package="cartographer_ros",
        executable="occupancy_grid_node",
        output="screen",
        name="occupancy_grid_node",
        parameters=[{"use_sim_time": True}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"]
    )
    
    rviz = Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=["-d", rviz_config_file],
        output= "screen",
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        cartographer,
        grid,
        rviz
    ])
