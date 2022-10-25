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
from ament_index_python.packages import get_package_share_directory

# import launch
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
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
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[generate_robot_description()],
    )
    return LaunchDescription([robot_state_publisher])
