from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

URDF_PATH = os.path.expanduser("~/pyclops/ros_ws/src/robot_description/urdf/robot.urdf")

def generate_launch_description():
    ld = LaunchDescription()

    robot_description = ParameterValue(
        Command(['cat ', URDF_PATH]),
        value_type=str
    )

    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description
            }]
    )
    ld.add_action(state_publisher_node)

    key_input_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
    )
    ld.add_action(key_input_node)

    steering_node = Node(
        package="pyclops_controller",
        executable="teleop_node"
    )
    ld.add_action(steering_node)

    drive_node = Node(
        package="pyclops_controller",
        executable="drive_node"
    )
    ld.add_action(drive_node)

    return ld