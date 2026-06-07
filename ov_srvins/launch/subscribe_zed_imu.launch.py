from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(
        name="namespace", default_value="ov_srvins", description="namespace"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value=os.path.join(
            get_package_share_directory("ov_srvins"),
            "config", "zed_imu", "estimator_config.yaml",
        ),
        description="path to estimator_config.yaml",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    ),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)

    if not os.path.isfile(config_path):
        return [LogInfo(msg="ERROR: config_path '{}' does not exist.".format(config_path))]

    # ZED IMU publishes standard sensor_msgs/Imu — no converter needed
    estimator = Node(
        package="ov_srvins",
        executable="run_subscribe_msckf",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"config_path": config_path},
            {"use_unitree_imu": False},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d" + os.path.join(
                get_package_share_directory("ov_srvins"), "launch", "display_ros2.rviz"
            ),
            "--ros-args", "--log-level", "warn",
        ],
    )

    return [estimator, rviz]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
