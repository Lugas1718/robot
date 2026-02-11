import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ganti 'username' dengan user Ubuntu di NUC kamu
    urdf_path = '/home/username/ROBOT0302/description/robot.urdf'

    return LaunchDescription([
        # 1. Micro-ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyESP32', '-b', '921600']
        ),

        # 2. Driver Lidar A1M8
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyLidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),

        # 3. Robot State Publisher (PENTING: Ini pengganti Static TF manual)
        # Node ini akan membaca file URDF dan mengurus SEMUA TF (laser, wheel, imu) sekaligus.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # 4. IMU Filter Madgwick
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False # Biarkan ESP32 yang publish TF odom->base
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data_filtered')
            ]
        ),
    ])