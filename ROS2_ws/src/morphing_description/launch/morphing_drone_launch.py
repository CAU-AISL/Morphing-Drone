import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SDF 모델이 들어있는 패키지
    pkg = get_package_share_directory('morphing_description')
    sdf = os.path.join(pkg, 'model', 'Morphing_drone.sdf')

    return LaunchDescription([
        # 1) Gazebo 실행 (ROS API 플러그인 없이도, 
        #    플러그인이 자체적으로 rclcpp spin() 을 띄우므로 OK)
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so','-s', 'libgazebo_ros_init.so', sdf],
            output='screen'
        ),

        # 2) ROS2 퍼블리셔 노드: /my_drone/motor_speeds 토픽에 속도 전송
        Node(
            package='motor_speeds_pub',      # 본인이 만든 퍼블리셔 패키지 이름
            executable='motor_speeds_pub',   # 해당 노드 실행 파일 이름
            name='motor_speeds_pub',
            output='screen',
            parameters=[  # 토픽명·속도 초기값 등 파라미터 주고 싶으면 여기에
                # {'publish_topic': '/my_drone/motor_speeds'},
                # {'motor_rate_hz': 10.0},
            ]
        ),
        
        # 3) main_controller 노드
        Node(
            package='morphing_drone_control',
            executable='main_controller',
            name='morphing_drone_controller',
            output='screen',
        )
    ])
