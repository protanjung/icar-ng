import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
    ])
