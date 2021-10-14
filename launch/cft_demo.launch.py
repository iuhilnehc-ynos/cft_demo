# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a parameter_blackboard of demo_nodes_cpp and a subscriber_member_function_with_content_filtered_topic of cft_demo."""

from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
          package='cft_demo',
          executable='subscriber_member_function_with_content_filtered_topic',
          output='screen',
          emulate_tty=True),
        launch.actions.TimerAction(
            period=2.,
            actions=[
              launch_ros.actions.Node(
                  package='demo_nodes_cpp', executable='parameter_blackboard', output='screen'),
            ],
        ),
    ])
