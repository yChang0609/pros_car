from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pros_car_py.env import SERIAL_DEV_DEFAULT
print("SERIAL_DEV_DEFAULT:",SERIAL_DEV_DEFAULT)
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', 
            default_value=SERIAL_DEV_DEFAULT, 
            description='Serial port for the car nodes'
        ),
        Node(
            package='pros_car_py',  # Replace with your package name
            executable='carA_reader',
            name='carA_reader_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen'
        ),
        Node(
            package='pros_car_py',  # Replace with your package name
            executable='carA_writer',
            name='carA_writer_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen'
        ),
        Node(
            package='pros_car_py',  # Replace with your package name
            executable='carA_random',
            name='carA_random_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen'
        )
        
    ])
