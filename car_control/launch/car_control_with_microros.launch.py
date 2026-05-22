#!/usr/bin/env python3
"""
Launch file to run car_control_node with Micro-ROS Agent.

Connects microcontroller via serial interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_EXAMPLE_USAGE = """Example usage:
  ros2 launch car_control car_control_with_microros.launch.py

  ros2 launch car_control car_control_with_microros.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=921600
"""


def _validate_launch_args(context, *_args, **_kwargs):
    port = LaunchConfiguration('serial_port').perform(context).strip()
    if not port:
        raise RuntimeError(
            "Launch argument 'serial_port' is required (non-empty serial device path).\n\n"
            + _EXAMPLE_USAGE
        )

    baud = LaunchConfiguration('baud_rate').perform(context).strip()
    if not baud.isdigit() or int(baud) <= 0:
        raise RuntimeError(
            f"Launch argument 'baud_rate' must be a positive integer (got {baud!r}).\n\n"
            + _EXAMPLE_USAGE
        )

    return []


def generate_launch_description():
    # Declare launch arguments (serial_port default empty; validated in OpaqueFunction with example text)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='Serial device path for Micro-ROS Agent, e.g. /dev/ttyTHS1 or /dev/ttyUSB0',
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='460800',
        description='Serial baud rate for Micro-ROS Agent'
    )

    use_docker_arg = DeclareLaunchArgument(
        'use_docker',
        default_value='false',
        description='Use Docker for Micro-ROS Agent (true/false)'
    )

    agent_verbose_arg = DeclareLaunchArgument(
        'agent_verbose',
        default_value='0',
        description='Micro-ROS Agent log verbosity (0=quiet … 6=debug hex dumps)',
    )

    # Get launch configuration
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    agent_verbose = LaunchConfiguration('agent_verbose')

    # Micro-ROS Agent Node (runs from source)
    microros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '-D', serial_port, '-b', baud_rate, '-v', agent_verbose],
        output='screen',
        name='micro_ros_agent',
        emulate_tty=True
    )

    # Car Control Node
    car_control_node = Node(
        package='car_control',
        executable='car_control_node',
        output='screen',
        name='car_control_node',
    )

    validate_args = OpaqueFunction(function=_validate_launch_args)

    # Return launch description with all nodes
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        use_docker_arg,
        agent_verbose_arg,
        validate_args,
        microros_agent_node,
        car_control_node,
    ])
