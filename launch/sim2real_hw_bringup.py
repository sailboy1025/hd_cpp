from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hd_cpp',
            executable='inverse3_cpp_node',
            name='inverse3_cpp_node',
            arguments=['-gc'],
            output='screen'
        ),
        Node(
            package='delcomp_cpp_pkg',
            executable='abs_teleop',
            name='abs_teleop_node',
            remappings=[
                ('viper_pose', 'inv3_pose')
            ],
            output='screen'
        ),
        Node(
            package='delcomp_cpp_pkg',
            executable='delay_node',
            name='delay_node',
            output='screen'
        )
    ])
