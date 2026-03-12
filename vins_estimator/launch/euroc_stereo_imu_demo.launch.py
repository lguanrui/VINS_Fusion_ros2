from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("vins")
    default_config = PathJoinSubstitution(
        [package_share, "config", "euroc", "euroc_stereo_imu_config.yaml"]
    )
    default_rviz = PathJoinSubstitution([package_share, "config", "vins_rviz.rviz"])
    default_log_dir = str(Path.home() / ".ros" / "vins_fusion_logs")

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    use_rviz = LaunchConfiguration("use_rviz")
    use_loop_fusion = LaunchConfiguration("use_loop_fusion")
    play_bag = LaunchConfiguration("play_bag")
    world_frame = LaunchConfiguration("world_frame")
    world_tf_child_frame = LaunchConfiguration("world_tf_child_frame")

    vins_node = ExecuteProcess(
        cmd=["ros2", "run", "vins", "vins_node", config_file],
        output="screen",
    )

    loop_fusion_node = ExecuteProcess(
        cmd=["ros2", "run", "loop_fusion", "loop_fusion_node", config_file],
        output="screen",
        condition=IfCondition(use_loop_fusion),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    world_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_tf_publisher",
        arguments=["0", "0", "0", "0", "0", "0", world_frame, world_tf_child_frame],
        output="screen",
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--rate",
            bag_rate,
            "--disable-keyboard-controls",
        ],
        output="screen",
    )

    bag_group = GroupAction(
        condition=IfCondition(play_bag),
        actions=[
            TimerAction(period=2.0, actions=[bag_play]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bag_play,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="ros2 bag play completed, shutting down EuRoC demo"
                            )
                        )
                    ],
                )
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="VINS/loop_fusion config yaml.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 configuration file.",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="ROS2 bag directory to play for the EuRoC demo.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
                description="Playback rate passed to ros2 bag play.",
            ),
            DeclareLaunchArgument(
                "play_bag",
                default_value="false",
                description="If true, launch ros2 bag play after the nodes start.",
            ),
            DeclareLaunchArgument(
                "use_loop_fusion",
                default_value="true",
                description="If true, also launch loop_fusion_node.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="If true, launch RViz2 with the packaged config.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="world",
                description="Parent frame for the default static world TF.",
            ),
            DeclareLaunchArgument(
                "world_tf_child_frame",
                default_value="world_anchor",
                description="Child frame used to materialize the world frame in TF.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value=default_log_dir,
                description="Directory used for ROS 2 launch logs.",
            ),
            SetEnvironmentVariable("ROS_LOG_DIR", LaunchConfiguration("log_dir")),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            world_tf_node,
            vins_node,
            loop_fusion_node,
            rviz_node,
            bag_group,
        ]
    )
