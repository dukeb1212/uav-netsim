import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_nodes(context, *args, **kwargs):
    # Retrieve configuration values from launch context
    node_type = LaunchConfiguration('node').perform(context)
    target_ip = LaunchConfiguration('target_ip').perform(context)

    actions = []

    # Define Ping Publisher Node
    ping_pub_node = Node(
        package='uav_gs',
        executable='ping_publisher.py',
        name='ping_pub_node',
        output='screen',
        parameters=[{'target_ip': target_ip}]
    )

    # Define Ping Subscriber Node
    ping_sub_node = Node(
        package='uav_gs',
        executable='ping_subscriber.py',
        name='ping_sub_node',
        output='screen'
    )

    # Conditional node launching
    if node_type == 'pub':
        actions.append(ping_pub_node)
        actions.append(LogInfo(msg=f"Launching Ping Publisher Node...\nStart pinging host: {target_ip}"))
    elif node_type == 'sub':
        actions.append(LogInfo(msg="Launching Ping Subscriber Node..."))
        actions.append(ping_sub_node)
    else:
        actions.append(LogInfo(msg="Invalid node type specified. Use 'pub' or 'sub'."))

    return actions

def generate_launch_description():
    # Declare arguments for node type and target IP
    node_type = DeclareLaunchArgument(
        'node',
        default_value='pub',
        description='Node type to launch: pub or sub'
    )
    
    target_ip = DeclareLaunchArgument(
        'target_ip',
        default_value='10.0.0.1',
        description='Target IP address for pinging'
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(node_type)
    ld.add_action(target_ip)
    ld.add_action(OpaqueFunction(function=launch_nodes))

    return ld
