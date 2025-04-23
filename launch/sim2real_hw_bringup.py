from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hd_cpp',
            executable='inverse3_cpp_node',
            name='inverse3_cpp_node',
            arguments=['-gc'],
            namespace='inv3',
            output='screen'
        ),
        Node(
            package='delcomp_cpp_pkg',
            executable='abs_teleop',
            name='abs_teleop_node',
            remappings=[
                ('viper_pose', '/inv3/pose')
            ],
            parameters=[{
                'teleop_param': 0.4
            }],
            output='screen'
        ),
        Node(
            package='delcomp_cpp_pkg',
            executable='delay_node',
            name='delay_node',
            remappings=[
                ('joy', '/inv3/button')
            ],
            parameters=[{
                'delay_time': 0
            }],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            remappings=[
                ('joy', 'grasp_cmd')
            ],
            output='screen'
        )
    ])
