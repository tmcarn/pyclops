import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package name
    # pkg_name = 'pyclops_controller'

    ld = LaunchDescription()
    
    # Start Publishing Robot State
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('pyclops_launch'),'launch','rsp.launch.py'
                    )])
            )
    ld.add_action(rsp)

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',  # Opens in new terminal window
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(teleop_node)

    ik_node = Node(
        package='pyclops_controller',
        executable='ik_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(ik_node)

    diff_drive_controller = Node(
        package='pyclops_controller',
        executable='drive_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(diff_drive_controller)

    # Initialize Environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r -v4 empty.sdf'}.items()
    )
    ld.add_action(gazebo)

    # Spawn in Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pyclops',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    ld.add_action(spawn_entity)

    # Enable ROS2<-->Gazebo Topic Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/pyclops/joint/left_wheel_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/pyclops/joint/right_wheel_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ],
        output='screen'
    )
    ld.add_action(bridge)

    return ld



