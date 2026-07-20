# Copyright 2022 eSOL Co.,Ltd.
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

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    # Set the path to the parameter file
    config_file_path = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'params_ether.yaml'
    )

    # Load the parameter file
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # Launch urg_node2 as a lifecycle node
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params, {
            'use_hte': LaunchConfiguration('use_hte'),
            'hte_gpio_chip': LaunchConfiguration('hte_gpio_chip'),
            'hte_gpio_offset': LaunchConfiguration('hte_gpio_offset'),
        }],
        namespace='',
        output='screen',
    )

    # Transition from Unconfigured to Inactive state (executed when auto_start is true)
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Transition from Inactive to Active state (executed when auto_start is true)
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # About the parameters:
    # auto_start      : Automatically transition to Active state at startup (default)true
    # node_name       : Node name (default)"urg_node2"
    # scan_topic_name : Topic name (default)"scan" *multiecho not supported*
    # use_hte         : Whether to enable HTE (Hardware Timestamp Engine) (default)true
    # hte_gpio_chip   : GPIO chip to use for HTE (default)"/dev/gpiochip1"
    # hte_gpio_offset : GPIO pin offset to use for HTE (default)0
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('node_name', default_value='urg_node2'),
        DeclareLaunchArgument('scan_topic_name', default_value='scan'),
        DeclareLaunchArgument('use_hte', default_value='true'),
        DeclareLaunchArgument('hte_gpio_chip', default_value='/dev/gpiochip1'),
        DeclareLaunchArgument('hte_gpio_offset', default_value='0'),
        lifecycle_node,
        urg_node2_node_configure_event_handler,
        urg_node2_node_activate_event_handler,
    ])
