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

import imp
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
import xacro
from launch import LaunchDescription
import launch


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
    webots = WebotsLauncher(
        world=os.path.join(
            get_package_share_directory("robotis_op2_description"),
            "world",
            "robocup.wbt",
        )
    )
    ros2_supervisor = Ros2SupervisorLauncher()
    controller_url = "ipc://1234/"
    driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url + "ROBOTIS_OP2"},
        parameters=[generate_robot_description()],
    )
    handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    return LaunchDescription([webots, ros2_supervisor, driver, handler])
