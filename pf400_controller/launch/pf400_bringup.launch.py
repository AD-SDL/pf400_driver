from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pf400_manager = Node(
        package="pf400_controller",
        executable="pf400_manager",
        output='screen',
        parameters=[{"name": "pf400_node"}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    pf400_transfer_handler = Node(
        package="pf400_controller",
        executable="pf400_transfer_handler",
        output='screen',
        parameters=[{"name": "pf400_node"}],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    ld.add_action(pf400_manager)
    ld.add_action(pf400_transfer_handler)
    return ld
