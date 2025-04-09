from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(get_package_share_directory('open_loop_controller'), 'config', 'config.yaml')

    node1 = Node(package='turtlesim',
                       executable='turtlesim_node',
                       name="turtlesim",
                       )
        
    node2 = Node(package='open_loop_controller',
                       executable='path_generator',
                       name="path_generator",
                       parameters=[config]
                       )
    
    node3 = Node(package='open_loop_controller',
                       executable='tracker_points',
                       name="tracker_points",
                       parameters=[config]
                       )
    
    
    
    l_d = LaunchDescription([node1, node2, node3])

    return l_d