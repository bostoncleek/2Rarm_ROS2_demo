import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    container = ComposableNodeContainer(
        name='arm_demo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='simple_arm',
                plugin='simple_arm::SimpleArm',
                name='simple_arm',
                parameters=[os.path.join(get_package_share_directory("simple_arm"), "config", "arm.yaml")])
        ], 
        output='screen',
    )

    return LaunchDescription([ container ])
    