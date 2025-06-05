import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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

        # 2) 컨트롤러 노드 (5초 지연 실행)
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='morphing_drone_control',
                    executable='main_controller',
                    name='morphing_drone_controller',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    # parameters=[os.path.join(pkg_desc, 'config', 'your_params.yaml')],
                )
            ]
        ),

        # 3) 모터 퍼블리셔 노드
        Node(
            package='motor_speeds_pub',
            executable='motor_speeds_pub',
            name='motor_speeds_pub',
            output='screen',parameters=[{'use_sim_time': True}],
        ),
    ])
