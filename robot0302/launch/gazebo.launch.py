import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Ambil path file URDF secara otomatis
    urdf_path = '/home/username/ROBOT0302/description/robot.urdf' # GANTI USERNAME KAMU

    return LaunchDescription([
        # 1. Robot State Publisher (Membaca file URDF agar robot muncul)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        # 2. Jalankan Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # 3. Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'esp32_robot'],
            output='screen'
        ),
    ])