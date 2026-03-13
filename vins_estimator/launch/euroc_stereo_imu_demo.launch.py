from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("vins")
    default_config = PathJoinSubstitution(
        [package_share, "config", "euroc", "euroc_stereo_imu_config.yaml"]
    )
    default_rviz = PathJoinSubstitution([package_share, "config", "vins_fusion.rviz"])
    default_log_dir = str(Path.home() / ".ros" / "vins_fusion_logs")
    default_record_bag = str(Path.home() / ".ros" / "vins_fusion_recordings" / "euroc_results")
    default_extra_library_path = PathJoinSubstitution(
        [
            package_share,
            "..",
            "..",
            "..",
            "..",
            "src",
            "vins_fusion",
            "third_party",
            "ceres-install",
            "lib",
        ]
    )

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    use_rviz = LaunchConfiguration("use_rviz")
    use_loop_fusion = LaunchConfiguration("use_loop_fusion")
    play_bag = LaunchConfiguration("play_bag")
    world_frame = LaunchConfiguration("world_frame")
    world_tf_child_frame = LaunchConfiguration("world_tf_child_frame")
    record_results = LaunchConfiguration("record_results")
    record_bag_path = LaunchConfiguration("record_bag_path")
    extra_library_path = LaunchConfiguration("extra_library_path")

    vins_component = ComposableNode(
        package="vins",
        plugin="vins::VinsEstimatorComponent",
        name="vins_estimator",
        namespace="vins_estimator",
        parameters=[{"config_file": config_file}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    loop_fusion_component = ComposableNode(
        package="loop_fusion",
        plugin="loop_fusion::LoopFusionComponent",
        name="loop_fusion",
        namespace="loop_fusion",
        parameters=[
            {"config_file": config_file},
            {"enable_keyboard_commands": False},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="vins_fusion_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[vins_component],
        output="screen",
    )

    load_loop_fusion = LoadComposableNodes(
        target_container="vins_fusion_container",
        composable_node_descriptions=[loop_fusion_component],
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

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            record_bag_path,
            "/vins_estimator/odometry",
            "/vins_estimator/path",
            "/loop_fusion/odometry_rect",
            "/loop_fusion/pose_graph_path",
        ],
        output="screen",
        condition=IfCondition(record_results),
    )

    bag_group = GroupAction(
        condition=IfCondition(play_bag),
        actions=[
            TimerAction(period=1.0, actions=[bag_record]),
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
                "record_results",
                default_value="false",
                description="If true, record VINS result topics into a ros2 bag.",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value=default_record_bag,
                description="Output directory for the recorded results bag.",
            ),
            DeclareLaunchArgument(
                "extra_library_path",
                default_value=default_extra_library_path,
                description="Extra library directory prepended to LD_LIBRARY_PATH for bundled deps.",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value=default_log_dir,
                description="Directory used for ROS 2 launch logs.",
            ),
            SetEnvironmentVariable("ROS_LOG_DIR", LaunchConfiguration("log_dir")),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    extra_library_path,
                    ":/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            world_tf_node,
            container,
            load_loop_fusion,
            rviz_node,
            bag_group,
        ]
    )
