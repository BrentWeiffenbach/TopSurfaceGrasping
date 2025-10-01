import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('top_surface_enviornment'),
        'worlds',
        'top_surface_grasping.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items(),
    )

    camera_urdf_path = os.path.join(
        get_package_share_directory('top_surface_enviornment'),
        'urdf',
        'camera.urdf'
    )
    robot_description = {'robot_description': open(camera_urdf_path).read()}

    spawn_camera = Node(package='gazebo_ros', executable="spawn_entity.py",
                        arguments=['-file', camera_urdf_path,
                                    '-entity','camera',
                                    '-x','-0.25',
                                    '-z','1.3',
                                    '-P','1.22'],
                        output='both' )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        gazebo,
        spawn_camera,
        robot_state_publisher
    ])