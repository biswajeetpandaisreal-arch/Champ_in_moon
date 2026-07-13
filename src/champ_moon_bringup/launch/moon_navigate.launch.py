import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moon_pkg_share = get_package_share_directory("champ_moon_bringup")

    default_map_path = os.path.join(moon_pkg_share, "maps", "moon_map.yaml")
    default_params_path = os.path.join(moon_pkg_share, "config", "nav2_params.yaml")
    default_rviz_path = os.path.join(moon_pkg_share, "rviz", "nav_view.rviz")

    declare_map = DeclareLaunchArgument("map", default_value=default_map_path)
    declare_params_file = DeclareLaunchArgument("params_file", default_value=default_params_path)
    declare_sim = DeclareLaunchArgument("sim", default_value="true")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="true")

    nav2_bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": LaunchConfiguration("sim"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
    )

    return LaunchDescription(
        [
            declare_map,
            declare_params_file,
            declare_sim,
            declare_rviz,
            nav2_bringup_ld,
            rviz_node,
        ]
    )
