import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SDF 모델 경로
    pkg_desc = get_package_share_directory('morphing_description')
    sdf_path = os.path.join(pkg_desc, 'model', 'Morphing_drone.sdf')

    return LaunchDescription([
        # 1) Gazebo + ROS2 플러그인
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so',
                sdf_path
            ],
            output='screen'
        ),

        # 2) 컨트롤러 노드
        Node(
            package='morphing_drone_control',
            executable='main_controller',
            name='morphing_drone_controller',
            output='screen',
            parameters=[{
                'bodyMass': 1.289,
                'armMass': 0.139,
                'armcmLength': 0.139113738822933,
                'armLength': 0.1595,
                'bodyLength': 0.114552,
                'Ixxa': 2.190839539142159e-04,
                'Ixya': -4.301600091384631e-05,
                'Ixza': 6.131159813951677e-05,
                'Iyya': 3.986596961705956e-04,
                'Iyza': 6.459346045048527e-05,
                'Izza': 3.329336946986987e-04,
                'Ixxb': 0.001313892,
                'Iyyb': 0.001655877,
                'Izzb': 0.002162122,
                'Ixzb': -1.835200000000000e-05,
                'ThrustCoeff': 1e-5,
                'DragCoeff': 1e-6
            }]
        ),

        # 3) 모터 퍼블리셔 노드
        Node(
            package='motor_speeds_pub',
            executable='motor_speeds_pub',
            name='motor_speeds_pub',
            output='screen',
        ),
    ])