from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}]
        ),
        Node(
            package='ur5e_joystick_control',
            executable='ur5e_inverse_kinematics',
            name='ur5e_inverse_kinematics',
            output='screen'
        )
    ])
