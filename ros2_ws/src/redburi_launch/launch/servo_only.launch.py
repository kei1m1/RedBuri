from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "redburi_arm", package_name="redburi_moveit"
    ).to_moveit_configs()
    servo_param = PathJoinSubstitution(
        [FindPackageShare("teleop"), "config", "servo.yaml"]
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_param,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    start_servo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([servo_node, start_servo])
