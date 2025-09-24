# Grasping using top surfaces

## Use Cases

1. Segment object in point cloud
    - Given full point cloud image, segment and return only the desired object or objects in the pointcloud
2. Shape from point cloud
    - Given a segmented point cloud (RGB and Depth data), return the shape of the 'top' of the object, and the object's center of mass projected to the 'top' shape.
3. Grasp generation
    - Given a 2D shape and parallel gripper maximum grip size, randomly sample a graspable posiiton on the object, generating a grasp matrix for the grasp.
4. Grasp Quality
    - Given two points of contact on a 2D shape, determine the grasp quality metrics of Minimum Singular Value, Isotropy, and Ellipsoid volume.
5. Grasp Search
    - Use hill climbing or other optimiztion algorithim to resample and move grasp to better position until reached a local/global maximum for the quality metrics.
