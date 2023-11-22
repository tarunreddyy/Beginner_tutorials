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
    output_bag_directory = os.path.join(os.getcwd(), 'output_bag')

    return LaunchDescription([
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable or disable bag recording.'
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record_bag')),
            cmd=['ros2', 'bag', 'record', '-a', '-o', output_bag_directory],
            output='screen'
        ),

        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker_node',
        ),

        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('record_bag')),
                    cmd=['pkill', '-f', 'ros2 bag record'],
                    output='screen'
                )
            ]
        ),
    ])
