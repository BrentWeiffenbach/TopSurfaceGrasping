from setuptools import find_packages, setup

package_name = 'top_surface_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brent',
    maintainer_email='rweiff2022@gmail.com',
    description='Manager node for top surface grasping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pipeline_manager = top_surface_manager.grasp_pipeline_manager:main',
        ],
    },
)
