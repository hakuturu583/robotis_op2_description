# Copyright (c) 2022 Masaya Kataoka
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

from click import launch
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
import xacro


def generate_robot_description():
    share_dir_path = os.path.join(
        get_package_share_directory("robotis_op2_description")
    )
    xacro_path = ""
    xacro_path = os.path.join(share_dir_path, "urdf", "robotis_op2.urdf.xacro")
    doc = xacro.process_file(xacro_path)
    robot_description = {"robot_description": doc.toxml()}
    return robot_description


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz", default=True)
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value=launch_rviz,
        description="If true, launch with given rviz configuration.",
    )
    mode = LaunchConfiguration("mode", default="simulation")
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value=mode,
        description="Please select launch mode, simulation, joint_state_publisher is available.",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[generate_robot_description()],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        condition=LaunchConfigurationEquals("mode", "joint_state_publisher"),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output={"stderr": "log", "stdout": "log"},
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("robotis_op2_description"),
                "rviz",
                "robot.rviz",
            ),
        ],
    )
    # control_node = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[
    #        generate_robot_description(),
    #        os.path.join(
    #            get_package_share_directory("robotis_op2_description"),
    #            "config",
    #            "controllers.yaml",
    #        ),
    #    ],
    #    output={"stdout": "screen", "stderr": "screen"},
    #    condition=LaunchConfigurationEquals("mode", "simulation"),
    # )
    return LaunchDescription(
        [
            robot_state_publisher,
            rviz,
            launch_rviz_arg,
            joint_state_publisher,
            mode_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("robotis_op2_description"),
                        "/launch/webots.launch.py",
                    ]
                ),
                condition=LaunchConfigurationEquals("mode", "simulation"),
            ),
            # control_node,
        ]
    )
