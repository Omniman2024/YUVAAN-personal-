from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the joy node to read from a joystick device.
    """
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0'}
            ]
        )
    ])
