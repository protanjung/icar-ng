import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    realsense2_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        parameters=[{'enable_accel': True,
                     'enable_gyro': True,
                     'unite_imu_method': 2,
                     'align_depth.enable': True,
                     'initial_reset': True}],
        remappings=[('/imu', '/imu_raw')]
    )

    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{'use_mag': False}],
        remappings=[('/imu/data_raw', '/imu_raw'),
                    ('/imu/data', '/imu_filtered')]
    )

    camera_dataset_labeler = Node(
        package='icar_utilities',
        executable='camera_dateset_labeler',
        name='camera_dateset_labeler',
        output='screen'
    )

    return LaunchDescription([
        realsense2_camera_node,
        imu_filter_madgwick_node,
        camera_dataset_labeler,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=camera_dataset_labeler,
                on_exit=EmitEvent(event=Shutdown(reason='camera_dataset_labeler exited'))
            )
        )
    ])
