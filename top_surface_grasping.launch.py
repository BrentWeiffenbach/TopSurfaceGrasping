import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Paths to RViz config and Gazebo world
    rviz_config = os.path.join(base_dir, 'top_surface_grasping.rviz')
    gazebo_world = os.path.join(base_dir, 'top_surface_grasping.world')

    return LaunchDescription([
        # Segmentation Node
        Node(
            package='top_surface_segmentation',
            executable='segmentation_node',
            name='segmentation_node',
            output='screen'
        ),
        # Object Detector Node
        Node(
            package='top_surface_object_detector',
            executable='object_detector_node',
            name='object_detector_node',
            output='screen'
        ),
        # Geometry Extraction Node
        Node(
            package='top_surface_geometry',
            executable='geometry_extraction_node',
            name='geometry_extraction_node',
            output='screen'
        ),
        # Grasp Pipeline Manager Node
        Node(
            package='top_surface_manager',
            executable='grasp_pipeline_manager',
            name='grasp_pipeline_manager',
            output='screen'
        ),
        # Launch RViz with config
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        ),
        # Launch Gazebo with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world],
            output='screen'
        ),
    ])