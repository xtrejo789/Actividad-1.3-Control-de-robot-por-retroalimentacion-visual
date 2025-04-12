#Packages to get address of the YAML file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Get the address of the YAML File
    config = os.path.join(get_package_share_directory('act1_3'), 'config', 'params.yaml')
    movement_node = Node(name="movement",
                       package='act1_3',
                       executable='movement',
                       emulate_tty=True,
                       output='screen',
                       parameters=[config],
                       )
    sphere_node = Node(name="sphere",
                       package='act1_3',
                       executable='sphere',
                       emulate_tty=True,
                       output='screen',
                       )
    teleop_node = Node(name="teleop",
                          package='act1_3',
                          executable='teleop',
                          emulate_tty=True,
                          output='screen',
                          )
    
    l_d = LaunchDescription([movement_node, sphere_node, teleop_node])

    return l_d