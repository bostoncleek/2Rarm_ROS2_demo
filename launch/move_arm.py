import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # robot_urdf_file = os.path.join(get_package_share_directory("simple_arm"), "urdf", "arm.urdf")
    robot_urdf_file = os.path.join(get_package_share_directory("moveit_resources_panda_description"), "urdf", "panda.urdf")
    print("urdf_file_name : {}".format(robot_urdf_file))

    robot_description = {'robot_description' : robot_urdf_file}


    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', os.path.join(get_package_share_directory("simple_arm"), "config", "demo.rviz")],
                     parameters=[robot_description])

    # robot_state_publisher = Node(package='robot_state_publisher',
    #                              executable='robot_state_publisher',
    #                              name='robot_state_publisher',
    #                              output='screen',
    #                              parameters=[{'robot_description':Command(['xacro',' ', robot_urdf_file]) }] ) 

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
                parameters=[robot_description])
        ], 
        output='screen',
    )


    # container = ComposableNodeContainer(
    #     name='arm_demo_container',
    #     namespace='/',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='simple_arm',
    #             plugin='simple_arm::SimpleArm',
    #             name='simple_arm',
    #             parameters=[os.path.join(get_package_share_directory("simple_arm"), "config", "arm.yaml")])
    #     ], 
    #     output='screen',
    # )

    return LaunchDescription([ container, rviz_node ])