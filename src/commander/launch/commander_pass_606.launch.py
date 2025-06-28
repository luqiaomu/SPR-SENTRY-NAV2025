import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 status_wz_publisher 节点
        Node(
            package='commander',
            executable='pass_hole_606',
            name='status_wz_publisher',
            namespace='red_standard_robot1',
            remappings=[
                ('/tf', '/red_standard_robot1/tf'),
                ('/tf_static', '/red_standard_robot1/tf_static')
            ],
            output='screen'
        ),
 
    ])
