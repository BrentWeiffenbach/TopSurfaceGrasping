# Grasping using top surfaces

## Use Cases

1. Object Detector
    - Given an RGB image, create 2D detection boxes for all objects in the image.
2. Segment object in point cloud
    - Given a list of detection boxes and the RGB image create segmentaion masks for each object, and create segmented point clouds for each object.
3. Shape from point cloud
    - Given a list of segmented point clouds (RGB and Depth data), return the shape of the 'top' of each object, and the object's center of mass projected to the 'top' shape.
4. Grasp generation
    - Given a 2D shape and parallel gripper maximum grip size, randomly sample a graspable posiiton on the object, generating a grasp matrix for the grasp.
5. Grasp Quality
    - Given two points of contact on a 2D shape, determine the grasp quality metrics of Minimum Singular Value, Isotropy, and Ellipsoid volume.
6. Grasp Search
    - Use hill climbing or other optimiztion algorithim to resample and move grasp to better position until reached a local/global maximum for the quality metrics.
