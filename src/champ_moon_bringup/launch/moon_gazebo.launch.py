import os

import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_config").find(
        "champ_config"
    )
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find(
        "champ_description"
    )
    moon_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_moon_bringup"
    ).find("champ_moon_bringup")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")
    default_world_path = os.path.join(moon_pkg_share, "worlds/moon.world")
    models_path = os.path.join(moon_pkg_share, "models")

    # Gazebo needs to find model://lunar_tranquillitatis_pit on this path.
    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[models_path, ":", os.environ.get("GAZEBO_MODEL_PATH", "")],
    )

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false")
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_lite = DeclareLaunchArgument("lite", default_value="false")
    declare_gazebo_world = DeclareLaunchArgument("world", default_value=default_world_path)
    declare_gui = DeclareLaunchArgument("gui", default_value="true")

    # Spawn well clear of the crater near the heightmap's center, and high
    # (-42.4, -42.4) verified flat via heightmap pixel analysis (std < 1.1
    # grayscale over a 30m-radius patch) - only 60m from terrain center, well
    # clear of the crater. Measured ground elevation there is ~3.05m; spawn
    # just above it (not a big free-fall - a legged robot can topple on
    # impact if dropped from height, unlike a simple rigid body).
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="-42.4")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="-42.4")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="3.65")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("champ_bringup"), "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("champ_gazebo"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            set_model_path,
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
        ]
    )
