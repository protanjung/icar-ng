import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


# Collection of parameters for icar3 tested by Pandu Surya Tantra and Moh Ismarintan Zazuli.
# Should yo want to ask about these parameters, please contact them.
param_icar = {'icar.tf.rear_axle': [0.00, 0.00, 0.30, 0.00, 0.00, 0.00],
              'icar.tf.front_axle': [2.30, 0.00, 0.30, 0.00, 0.00, 0.00],
              'icar.tf.body': [1.15, 0.00, 1.18, 0.00, 0.00, 0.00],
              'icar.tf.gps': [2.30, 0.00, 2.05, 0.00, 0.00, 0.00],
              'icar.tf.lidar_front': [2.95, 0.00, 0.70, 0.00, -8.00, 179.00],
              'icar.tf.lidar_rearright': [-0.61, -0.81, 0.70, 0.00, 0.00, -131.00],
              'icar.tyre.width': 185,
              'icar.tyre.aspect_ratio': 60,
              'icar.tyre.rim_diameter': 15,
              'icar.body.length': 3.50,
              'icar.body.width': 1.50,
              'icar.body.height': 1.75,
              'icar.wheelbase': 2.30,
              'icar.conversion.encoder_pulse_to_meter': 0.000455,
              'icar.conversion.steering_pulse_to_radian': -0.000249,
              'icar.cf.alpha_xy': 0.999,
              'icar.cf.alpha_theta': 0.999}

# if 'GTK_PATH' environment variable is set, rviz2 will crash
# to avoid this, delete the variable before launching rviz2
if 'GTK_PATH' in os.environ:
    del os.environ['GTK_PATH']

# set rcutils logging format to only show severity and message
# also force colorized output to be enabled
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}]: {message}"
os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1"

icar_ng_data_path = os.path.join(os.environ['HOME'], 'icar-ng-data')


def generate_launch_description():
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', PathJoinSubstitution([icar_ng_data_path, 'bag', 'rosbag2_2023_10_26-18_59_07'])],
        name='rosbag')

    rviz2 = Node(package='rviz2',
                 executable='rviz2',
                 name='rviz2',
                 arguments=['-d', PathJoinSubstitution([icar_ng_data_path, 'rviz', 'icar3.rviz'])])

    realsense2_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        respawn=True,
        parameters=[{'enable_accel': True,
                     'enable_gyro': True,
                     'unite_imu_method': 2,
                     'align_depth.enable': True,
                     'initial_reset': True}],
        remappings=[('/imu', '/imu_raw')])

    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        respawn=True,
        parameters=[{'use_mag': False}],
        remappings=[('/imu/data_raw', '/imu_raw'),
                    ('/imu/data', '/imu_filtered')])

    io_stm32 = Node(
        package='icar_io',
        executable='io_stm32',
        name='io_stm32',
        respawn=True,
        parameters=[{'stm32.ip': '192.168.50.2',
                     'stm32.port': 9798}])

    io_gps = Node(
        package='icar_io',
        executable='io_gps',
        name='io_gps',
        respawn=True,
        parameters=[{'gps.port': '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',
                     'gps.baud': 115200,
                     'gps.origin_lat': -7.277463,
                     'gps.origin_lon': 112.797930}])

    io_lslidar_c16 = Node(
        package='icar_io',
        executable='io_lslidar_c16',
        name='io_lslidar_c16',
        respawn=True,
        parameters=[{'msop_port': 2368,
                     'difop_port': 2369,
                     'frame_id': 'lidar_front_link',
                     'azimuth_start': 90.0,
                     'azimuth_stop': 270.0,
                     'distance_min': 0.5,
                     'distance_max': 50.0}],
        remappings=[('/points_xyz', '/lidar_front/points_xyz'),
                    ('/points_xyzir', '/lidar_front/points_xyzir')])

    io_lslidar_n301 = Node(
        package='icar_io',
        executable='io_lslidar_n301',
        name='io_lslidar_n301',
        respawn=True,
        parameters=[{'msop_port': 2370,
                     'difop_port': 2371,
                     'frame_id': 'lidar_rearright_link',
                     'azimuth_start': 0.0,
                     'azimuth_stop': 360.0,
                     'distance_min': 0.5,
                     'distance_max': 50.0}],
        remappings=[('/points_xyz', '/lidar_rearright/points_xyz'),
                    ('/points_xyzi', '/lidar_rearright/points_xyzi')])

    transform_broadcaster = Node(
        package='icar_middleware',
        executable='transform_broadcaster',
        name='transform_broadcaster',
        respawn=True,
        parameters=[param_icar])

    pose_estimator = Node(
        package='icar_middleware',
        executable='pose_estimator',
        name='pose_estimator',
        respawn=True,
        parameters=[param_icar])

    lidar_transform = Node(
        package='icar_middleware',
        executable='lidar_transform',
        name='lidar_transform',
        respawn=True)

    road_segmentation = Node(
        package='icar_middleware',
        executable='road_segmentation',
        name='road_segmentation',
        respawn=True)

    routine = Node(
        package='icar_routine',
        executable='routine',
        name='routine',
        respawn=True,
        parameters=[param_icar])

    return LaunchDescription([
        rosbag,
        rviz2,
        # realsense2_camera_node,
        # imu_filter_madgwick_node,
        # io_stm32,
        # io_gps,
        # io_lslidar_c16,
        # io_lslidar_n301,
        transform_broadcaster,
        pose_estimator,
        lidar_transform,
        road_segmentation,
        routine
    ])
