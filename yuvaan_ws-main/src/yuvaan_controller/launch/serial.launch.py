from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the rosserial node for serial communication.
    """
    return LaunchDescription([
        Node(
            package='rosserial_python',
            executable='serial_node.py',
            name='rosserial_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baud_rate': 115200}
            ]
        )
    ])
