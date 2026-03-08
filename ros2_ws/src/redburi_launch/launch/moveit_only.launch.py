from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "redburi_arm", package_name="redburi_moveit"
    ).to_moveit_configs()
    moveit_share = FindPackageShare("redburi_moveit")

    static_tf_launch = PathJoinSubstitution(
        [moveit_share, "launch", "static_virtual_joint_tfs.launch.py"]
    )
    rsp_launch = PathJoinSubstitution(
        [moveit_share, "launch", "rsp.launch.py"]
    )
    move_group_launch = PathJoinSubstitution(
        [moveit_share, "launch", "move_group.launch.py"]
    )
    moveit_rviz_launch = PathJoinSubstitution(
        [moveit_share, "launch", "moveit_rviz.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(static_tf_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch)
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                parameters=[moveit_config.robot_description],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_rviz_launch)
            ),
        ]
    )
