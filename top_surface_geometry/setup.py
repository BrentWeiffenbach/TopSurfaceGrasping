from setuptools import find_packages, setup
import os

package_name = 'top_surface_geometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'),
            [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.launch.py')]),
        # Add config files
        (os.path.join('share', package_name, 'config'),
            [os.path.join('config', f) for f in os.listdir('config') if f.endswith('.rviz')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brent',
    maintainer_email='rweiff2022@gmail.com',
    description='Geometry utilities for top surface grasping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geometry_extraction_node = top_surface_geometry.geometry_extraction_node:main',
            'cube_cloud_publisher_node = top_surface_geometry.cube_cloud_publisher:main',
            'test_client_node = top_surface_geometry.test_client:main',
        ],
    },
)
