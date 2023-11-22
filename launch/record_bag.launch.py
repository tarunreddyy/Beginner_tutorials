# Copyright 2023 Tarun Trilokesh
#
# Licensed under the MIT License (MIT); you may not use this file except in
# compliance with the License. You may obtain a copy of the License at
# https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
@file record_bag.launch.py
@author Tarun Trilokesh
@date 11/17/2023
@version 1.0

@brief Launch file to record ROS 2 bag and run listener node.

This launch file runs the listener node of the beginner_tutorials package.
It includes an optional argument to start ROS 2 bag recording for a duration of
approximately 15 seconds and stores the output in a specified directory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the output directory for the bag files
    output_bag_directory = os.path.join(os.getcwd(), 'output_bag')

    return LaunchDescription([
        # Declare launch argument for bag recording
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable or disable bag recording.'
        ),

        # Conditionally start bag recording based on the record_bag argument
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-a', '-o', output_bag_directory],
            output='screen'
        ),

        # Listener node
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener_node',
        ),

        # Stop bag recording after 15 seconds
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    condition=LaunchConfiguration('record_bag'),
                    cmd=['pkill', '-f', 'ros2 bag record'],
                    output='screen'
                )
            ]
        ),
    ])
