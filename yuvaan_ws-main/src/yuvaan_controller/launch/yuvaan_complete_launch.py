# In your package's launch/ directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Command to run the rosserial node
    serial_node = Node(
        package='rosserial_python',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0'}]
    )

    # Command to run your controller node
    controller_node = Node(
        package='yuvaan_controller',
        executable='controller.py', # Make sure this script is executable and installed
        name='controller_node',
        output='screen'
    )

    # Command to launch rosbridge
    rosbridge_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    # Command to run the Flask video streamer
    # Note: Assumes app.py is installed to the package's lib directory
    app_py_node = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/app.py')],
        output='screen'
    )
    
    # Command to run the SocketIO bridge
    test2_py_node = Node(
        package='yuvaan_controller',
        executable='test2.py',
        name='socketio_bridge',
        output='screen'
    )

    return LaunchDescription([
        serial_node,
        controller_node,
        rosbridge_launch,
        app_py_node,
        test2_py_node
    ])
