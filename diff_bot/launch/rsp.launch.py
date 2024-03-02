import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file_name = 'robot.urdf'
    package_name= "diff_bot"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    # path_to_our_package/urdf/urdf_file
    robot_desc_path = os.path.join(get_package_share_directory(package_name), "urdf", urdf_file_name)
    
    # Robot State Publisher
    urdf_content = open(robot_desc_path).read()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        parameters=[{'use_sim_time': True, 'robot_description': urdf_content}],
        output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node
        ]
    )