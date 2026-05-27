import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    fake_joints_arg = DeclareLaunchArgument(
        'fake_joints',
        default_value='true',
        description='Publish fake joint states if true'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lbr',
        description='Robot name for namespacing'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    # Get the directory of this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_file = os.path.join(launch_file_dir, 'iiwa_robotiq.urdf')
    
    # Read URDF file and fix mesh paths
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Replace relative mesh paths with absolute paths
    robot_desc = robot_desc.replace(
        'filename="lbr_description/meshes/',
        f'filename="file://{launch_file_dir}/lbr_description/meshes/'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 30.0
        }],
        remappings=[('joint_states', '/lbr/joint_states')]
    )
    
    # Robot description publisher (publishes robot_description topic)
    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        name='robot_description_publisher',
        parameters=[{'robot_description': robot_desc}],
        remappings=[
            ('joint_states', '/dev/null'),  # Don't listen to joint states
            ('/tf', '/tf_unused'),          # Don't publish transforms
            ('/tf_static', '/tf_static_unused')  # Don't publish static transforms
        ]
    )
    
    # Joint state publisher (publishes zero positions for all joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_gui': False},
            {'rate': 10.0}
        ],
        remappings=[('joint_states', '/lbr/joint_states')],
        condition=IfCondition(LaunchConfiguration('fake_joints'))
    )
    
    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        fake_joints_arg,
        robot_name_arg,
        rviz_arg,
        robot_state_publisher,
        TimerAction(
            period=2.0,
            actions=[robot_description_publisher]
        ),
        TimerAction(
            period=4.0,
            actions=[joint_state_publisher]
        ),
        TimerAction(
            period=10.0,
            actions=[rviz_node]
        )
    ])
