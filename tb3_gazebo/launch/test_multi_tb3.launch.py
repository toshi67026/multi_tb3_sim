#!/usr/bin/env python3

import os

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NUM_ROBOTS = 2


def generate_launch_description() -> LaunchDescription:
    pkg_tb3_gazebo = get_package_share_directory("tb3_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world_file_name = "empty_world.model"
    world_file_path = os.path.join(pkg_tb3_gazebo, "worlds", world_file_name)

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world_file_path}.items(),
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py"))
    )

    xacro_file_path = os.path.join(pkg_tb3_gazebo, "urdf", "turtlebot3_burger.urdf.xacro")

    robot_node_group_list = []

    for i in range(NUM_ROBOTS):
        robot_name = "tb3_" + str(i)
        doc = xacro.process_file(xacro_file_path, mappings={"frame_prefix": robot_name + "/"})
        robot_desc = doc.toxml()

        theta = 2 * np.pi / NUM_ROBOTS * i
        robot_node_group_list.append(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),  # ロボット用のnemaspaceに閉じ込める
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        parameters=[{"robot_description": robot_desc}],
                    ),
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        # fmt: off
                        arguments=[
                            "-entity", robot_name,
                            "-robot_namespace", robot_name,
                            "-x", str(np.cos(theta)),
                            "-y", str(np.sin(theta)),
                            "-topic", "robot_description",
                        ]
                        # fmt: on
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        # fmt: off
                        arguments=["0", "0", "0", "0", "0", "0", "world", robot_name + "/odom"],
                        # fmt: on
                    ),
                ]
            )
        )

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(pkg_tb3_gazebo, "rviz", "test_multi_tb3.rviz"),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(rviz_node)

    for robot_node_group in robot_node_group_list:
        ld.add_action(robot_node_group)

    return ld
