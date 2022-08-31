#!/usr/bin/env python3
# gazeboへのモデル生成やrobot_state_publisherなどの併用テスト

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory("tb3_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world_file_name = "empty_world.model"
    world_file_path = os.path.join(pkg_tb3_gazebo, "worlds", world_file_name)

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_file_path}.items(),
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    xacro_file_path = os.path.join(
        pkg_tb3_gazebo, "urdf", "turtlebot3_burger.urdf.xacro"
    )

    robot_name = "test_tb3"

    # xacro;argを用いてxacroファイルにframe_prefixを渡してlink等に付与する
    doc = xacro.process_file(
        xacro_file_path, mappings={"frame_prefix": robot_name + "/"}
    )
    robot_desc = doc.toxml()
    # きれいに開業する場合
    # robot_desc = doc.toprettyxml(indent="  ")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=robot_name,
        parameters=[{"robot_description": robot_desc}],
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        namespace=robot_name,
        # fmt: off
        arguments=[
            "-entity", robot_name,
            "-robot_namespace", robot_name,
            "-topic", "robot_description",
        ],
        # fmt: on
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        namespace=robot_name,
        # fmt: off
        arguments=["0", "0", "0", "0", "0", "0", "world", robot_name + "/odom"],
        # fmt: on
    )

    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(pkg_tb3_gazebo, "rviz", "test_single_tb3.rviz"),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_launch)
    ld.add_action(gzclient_launch)
    ld.add_action(rviz_node)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(static_transform_publisher_node)

    return ld
