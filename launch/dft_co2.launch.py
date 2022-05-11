from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='francor_co2',
            namespace='francor_co2',
            executable='co2_sensor_interface',
            name='francor_co2',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "update_rate_hz": 1.0,
                    "serial_timeout_sec": 2.0,
                    "serial_name": "/dev/ttyCO2Sensor",
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])