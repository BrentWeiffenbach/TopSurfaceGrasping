# Grasping using top surfaces

![Project Architecture](<TopGraspingArchitecture.png>)

## Project Setup

Place this repository inside your ROS2 workspace src folder like a package. The folders within the TopSurfaceGrasping folder are the actual packages used to develop the ROS2 nodes we need.

From your ROS2 workspace, do a clean build:

```bash
rm -rf build/ install/ log/ && colcon build && source ./install  setup.bash
```

Open the TopSurfaceGrasping folder in VSCode. Make sure you have the [Python extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python) installed.  
To use Ruff for linting and formatting, install the [Ruff extension](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff) from the VSCode Marketplace.

If you have issues with pylance trying to import custom service messages into python, make sure you have built and sourced the workspace, with the correct python interpreter (same as ROS2), and you may need to create a `.vscode` folder and add this to `settings.json`:

```json
{
    "python.analysis.extraPaths": [
        "{ROS2_WORKSPACE}/install/top_surface_interfaces/lib/python3.10/site-packages"
    ]
}
```

The rviz and gazebo configs are just defaults we will need to develop into whatever we want later.

Template nodes and TODO services need to be finished

## Use Cases

1. Object Detector `[TODO]`
    - Given an RGB image, create 2D detection boxes for all objects in the image.
2. Segment object in point cloud `[TODO]`
    - Given a list of detection boxes and the RGB image, create segmentation masks for each object, and create segmented point clouds for each object.
3. Shape from point cloud `[TODO]`
    - Given a list of segmented point clouds (RGB and Depth data), return the shape of the 'top' of each object, and the object's center of mass projected to the 'top' shape.
4. Grasp generation `[TODO]`
    - Given a 2D shape and parallel gripper maximum grip size, randomly sample a graspable position on the object, generating a grasp matrix for the grasp.
5. Grasp Quality `[TODO]`
    - Given two points of contact on a 2D shape, determine the grasp quality metrics of Minimum Singular Value, Isotropy, and Ellipsoid volume.
6. Grasp Search `[TODO]`
    - Use hill climbing or other optimization algorithm to resample and move grasp to better position until reaching a local/global maximum for the quality metrics.
