from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Python 노드: main_controller_node
        Node(
            package='morphing_drone_control',
            executable='main_controller',
            name='morphing_drone_controller',
            output='screen',
        ),

        # C++ 노드: motor_speeds_pub_cpp
        Node(
            package='motor_speeds_pub',
            executable='motor_speeds_pub',
            name='motor_speeds_pub',
            output='screen',
        ),
    ])
