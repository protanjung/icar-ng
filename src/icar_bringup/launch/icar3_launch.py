import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}]: {message}"
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1"

    realsense2_camera_node = Node(package='realsense2_camera',
                                  executable='realsense2_camera_node',
                                  name='realsense2_camera_node',
                                  parameters=[{'enable_accel': True,
                                               'enable_gyro': True,
                                               'unite_imu_method': 2,
                                               'align_depth.enable': True,
                                               'initial_reset': True}],
                                  remappings=[('/imu', '/imu_raw')],
                                  respawn=True)

    imu_filter_madgwick_node = Node(package='imu_filter_madgwick',
                                    executable='imu_filter_madgwick_node',
                                    name='imu_filter_madgwick_node',
                                    parameters=[{'use_mag': False}],
                                    remappings=[('/imu/data_raw', '/imu_raw'),
                                                ('/imu/data', '/imu_filtered')],
                                    respawn=True)

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

    return LaunchDescription([realsense2_camera_node,
                              imu_filter_madgwick_node,
                              io_stm32,
                              io_gps])
