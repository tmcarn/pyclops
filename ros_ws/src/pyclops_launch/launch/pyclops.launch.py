from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

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