## @file
# @brief Launch file for initializing and running the talker and listener ROS nodes.
# @author Tarun Trilokesh
# @date 10/29/2023
# @version 1.0

import launch
from launch_ros.actions import Node

## @brief Function to generate the launch description for the talker and listener nodes.
#
# This function initializes the 'talker' and 'listener' nodes from the 'beginner_tutorials' package 
# and sets them up to be executed.
# 
# @return A LaunchDescription object containing the configurations for the nodes to be launched.
def generate_launch_description():
    return launch.LaunchDescription([
        ## Node initialization for the 'talker' node.
        Node(
            package='beginner_tutorials',     #< Name of the package containing the node.
            executable='talker',              #< Name of the node's executable.
            name='talker_node'                #< Name to be assigned to the initialized node.
        ),
        ## Node initialization for the 'listener' node.
        Node(
            package='beginner_tutorials',     #< Name of the package containing the node.
            executable='listener',            #< Name of the node's executable.
            name='listener_node'              #< Name to be assigned to the initialized node.
        )
    ])
