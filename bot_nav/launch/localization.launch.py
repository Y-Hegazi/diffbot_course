from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = "bot_nav"
    config_dir = os.path.join(get_package_share_directory(package_name), "config")
    localization_config_file = 'amcl_config.yaml'
    map_file = "obs_map.yaml"

    map_server = Node(

        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{'use_sim_time': True},
                    {"yaml_filename": os.path.join(config_dir, map_file)}],

    )

    localization = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        name="amcl",
        parameters=[os.path.join(config_dir, localization_config_file)],

    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name= "lifecycle_node",
        output="screen",
        parameters=[{"use_sim_time": True}, 
                    {"autostart": True},
                    {'node_names': ["map_server", "amcl"]}]
    )

    return LaunchDescription([
        map_server,
        localization,
        lifecycle_manager
    ])