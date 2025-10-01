import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Paths to RViz config and Gazebo world
    rviz_config = os.path.join(base_dir, "top_surface_grasping.rviz")
    gazebo_world = os.path.join(
        get_package_share_directory("top_surface_enviornment"),
        "worlds",
        "top_surface_grasping.world",
    )

    # Set GAZEBO_MODEL_PATH to include local models
    local_models_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "top_surface_enviornment", "models"
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=f"{local_models_path}:$GAZEBO_MODEL_PATH"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"world": gazebo_world}.items(),
    )

    camera_urdf_path = os.path.join(
        get_package_share_directory("top_surface_enviornment"), "urdf", "camera.urdf"
    )
    robot_description = {"robot_description": open(camera_urdf_path).read()}

    spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            camera_urdf_path,
            "-entity",
            "camera",
            "-x",
            "-0.25",
            "-z",
            "1.3",
            "-P",
            "1.22",
        ],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            set_gazebo_model_path,
            # Segmentation Node
            Node(
                package="top_surface_segmentation",
                executable="segmentation_node",
                name="segmentation_node",
                output="screen",
            ),
            # Object Detector Node
            Node(
                package="top_surface_object_detector",
                executable="object_detector_node",
                name="object_detector_node",
                output="screen",
            ),
            # Geometry Extraction Node
            Node(
                package="top_surface_geometry",
                executable="geometry_extraction_node",
                name="geometry_extraction_node",
                output="screen",
            ),
            # Grasp Pipeline Manager Node
            Node(
                package="top_surface_manager",
                executable="grasp_pipeline_manager",
                name="grasp_pipeline_manager",
                output="screen",
            ),
            # Launch RViz with config
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config], output="screen"),
            # Launch Gazebo
            gazebo,
            # Spawn camera in Gazebo
            spawn_camera,
            # Publish robot state for camera
            robot_state_publisher,
        ]
    )
