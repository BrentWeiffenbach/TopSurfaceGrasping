import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def camera_to_world(x, y, z, roll, pitch, yaw):
    # Reverse the roll, pitch, yaw transformation to get the original x, y, z
    # Assuming roll, pitch, yaw are in radians and represent intrinsic Tait-Bryan angles (XYZ)
    # Build rotation matrix from roll, pitch, yaw
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # Rotation matrix (R = Rz(yaw) * Ry(pitch) * Rx(roll))
    R = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]
    ]

    # Apply inverse rotation to (x, y, z)
    inv_R = [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]
    orig_x = inv_R[0][0]*x + inv_R[0][1]*y + inv_R[0][2]*z
    orig_y = inv_R[1][0]*x + inv_R[1][1]*y + inv_R[1][2]*z
    orig_z = inv_R[2][0]*x + inv_R[2][1]*y + inv_R[2][2]*z
    world_z = -orig_x
    world_y = orig_z
    world_x = orig_y

    return world_x, world_y, world_z

def rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]
    return R

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

    camera_x = -0.25
    camera_y = 0.0
    camera_z = 1.3
    camera_roll = 0
    camera_pitch = 1.22
    camera_yaw = 0

    spawn_camera = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", camera_urdf_path,
            "-entity", "camera",
            "-x", str(camera_x), "-y", str(camera_y), "-z", str(camera_z),
            "-R", str(camera_roll), "-P", str(camera_pitch), "-Y", str(camera_yaw),
        ],
        output="both",
    )
    
    world_x, world_y, world_z = camera_to_world(
        camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw
    )

    world_roll, world_pitch, world_yaw = 0,0,1.22

    static_world_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_tf_publisher",
        arguments=[
            str(world_x), str(world_y), str(world_z), str(world_roll), str(world_pitch), str(world_yaw), "camera_link", "world"
        ],
        output="screen",
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
            static_world_tf_publisher
        ]
    )
