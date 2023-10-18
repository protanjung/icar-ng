import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    io_stm32 = Node(package='icar_io',
                    executable='io_stm32',
                    name='io_stm32',
                    parameters=[{'stm32_ip': '192.168.50.2',
                                 'stm32_port': 9798}],
                    respawn=True)
    io_gps = Node(package='icar_io',
                  executable='io_gps',
                  name='io_gps',
                  respawn=True)

    return LaunchDescription([io_stm32,
                              io_gps])
