import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('top_surface_geometry'), 'config')
    rviz_config = os.path.join(config_dir, 'test_geometry_extraction.rviz')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'cube_frame'],
            output='screen',
        ),
        Node(
            package='top_surface_geometry',
            executable='cube_cloud_publisher_node',
            name='cube_cloud_publisher',
            output='screen',
        ),
        Node(
            package='top_surface_geometry',
            executable='geometry_extraction_node',
            name='geometry_extraction_node',
            output='screen',
        ),
        Node(
            package='top_surface_geometry',
            executable='test_client_node',
            name='test_client_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
