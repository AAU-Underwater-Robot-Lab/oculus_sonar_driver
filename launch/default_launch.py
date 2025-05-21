from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oculus_sonar_driver',
            executable='oculus_driver_node',
            name='oculus_driver',
            output='screen',
            parameters=['config/params.yaml']
        )
    ])
