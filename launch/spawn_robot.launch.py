from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'solar_robot',
                '-file', '/home/ronishbrh/ros2_ws/src/solar_cleaner_robot/urdf/solar_robot.urdf',
                '-x', '0', '-y', '0', '-z', '1'
            ],
            output='screen'
        )
    ])
