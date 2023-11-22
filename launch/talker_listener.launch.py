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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'new_string',
            default_value='Hello World!',  # Provide a sensible default
            description='New string to send to the change_string service.'
        ),
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker_node',
            # Additional configurations for the talker node...
        ),
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener_node',
            # Additional configurations for the listener node...
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'beginner_tutorials', 'change_string_client',
                        LaunchConfiguration('new_string')
                    ],
                    name='change_string_client_node',
                )
            ]
        ),
    ])
