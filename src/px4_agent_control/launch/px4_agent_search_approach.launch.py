import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create node
    px4_control_node = Node(
        package='px4_agent_control',
        executable='px4_agent_search_approach_node',
        name='px4_agent_search_approach',
        parameters=[{
            'height': 1.0,
            'mission_objective': "Go towards the humanoid robot at (1.35, 1.35).",
            'introspector_object': "Humanoid robot",
            'resend_command': True,
            'resend_size': 10
        }],
        remappings=[
            # Add any topic remappings
            # ('text0', 'custom_llm_command_topic'),
            # ('text1', 'custom_llm_response_topic')
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen'
    )

    return LaunchDescription([
        px4_control_node
    ])