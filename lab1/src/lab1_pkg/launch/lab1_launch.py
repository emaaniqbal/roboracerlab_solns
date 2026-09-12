from launch import LaunchDescription
from launch_ros.actions import Node

"""
This is the launch/test file for 
talker.py 
and relay.py
"""
# ref:https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

def generate_launch_description():
    # we are only worling with 2 nodes (1 talker, 1 relay)
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[{'d':1.0,'v':2.0}]
        ),
        Node(
            package='lab1_pkg',
            name='relay',
            executable='relay'
        )
    ])
