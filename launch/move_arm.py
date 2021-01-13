import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, 'r') as file:
#             return file.read()
#     except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
#         return None

def generate_launch_description():

    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('simple_arm'), 'urdf', 'arm.urdf.xacro'))

    robot_description = {'robot_description' : robot_description_config.toxml()}

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', os.path.join(get_package_share_directory("simple_arm"), "config", "demo.rviz")])

    container = ComposableNodeContainer(
        name='arm_demo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[robot_description]),
            ComposableNode(
                package='simple_arm',
                plugin='simple_arm::SimpleArm',
                name='simple_arm',
                parameters=[os.path.join(get_package_share_directory("simple_arm"), "config", "arm.yaml")])
        ], 
        output='screen',
    )

    # jsp_node = Node(package='joint_state_publisher_gui',
    #                 executable='joint_state_publisher_gui',
    #                 name='joint_state_publisher_gui',
    #                 parameters=[robot_description])


    return LaunchDescription([ container, rviz_node ])