#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import Polygon, Point, Point32
from sensor_msgs.msg import PointCloud2, Image
from top_surface_interfaces.srv import ExtractObjectGeometry
import numpy as np
from scipy.spatial import ConvexHull
import struct

def pointcloud2_to_xyz_array(cloud: PointCloud2) -> np.ndarray:
    # Assumes fields x, y, z are float32 and in order
    points = []
    point_step = cloud.point_step
    for i in range(cloud.width * cloud.height):
        offset = i * point_step
        x, y, z = struct.unpack_from('fff', cloud.data, offset)
        points.append([x, y, z])
    return np.array(points, dtype=np.float32)

class GeometryExtractionNode(Node):
    def __init__(self) -> None:
        super().__init__('geometry_extraction_node')
        
        # Create service
        self.service: Service = self.create_service(
            ExtractObjectGeometry,
            'extract_object_geometry',
            self.extract_geometry_callback
        )
        
        self.get_logger().info('Geometry extraction service ready')
    
    def extract_geometry_callback(
        self, 
        request: ExtractObjectGeometry.Request, 
        response: ExtractObjectGeometry.Response
    ) -> ExtractObjectGeometry.Response:
        mask = request.mask
        cloud = request.cloud
        assert isinstance(mask, Image)
        assert isinstance(cloud, PointCloud2)

        # Convert PointCloud2 to numpy array
        points_xyz = pointcloud2_to_xyz_array(cloud)

        # Convert mask to numpy array
        mask_np = np.array(mask.data, dtype=np.uint8).reshape(mask.height, mask.width)

        # Get indices of relevant pixels (mask > 0)
        relevant_indices = np.argwhere(mask_np > 0)

        # Map mask indices to point cloud indices (assuming organized cloud)
        relevant_points = []
        for y, x in relevant_indices:
            idx = y * mask.width + x
            if idx < points_xyz.shape[0]:
                relevant_points.append(points_xyz[idx])

        relevant_points = np.array(relevant_points)

        if relevant_points.size == 0:
            self.get_logger().warn('No points found in the masked region')
            response.geometry = Polygon()
            response.center = Point()
            return response
        
        # Find the top surface by selecting points in the highest Z range
        z_values = relevant_points[:, 2]
        z_max = np.max(z_values)
        z_thresh = 0.01
        top_mask = z_values > (z_max - z_thresh)
        top_points = relevant_points[top_mask]

        # Project top points to XY plane for 2D geometry extraction
        top_points_xy = top_points[:, :2]

        # Compute convex hull for the polygon
        if top_points_xy.shape[0] >= 3:
            hull = ConvexHull(top_points_xy)
            polygon_points = [Point32(x=float(top_points_xy[v, 0]), y=float(top_points_xy[v, 1]), z=0.0) for v in hull.vertices]
            response.geometry = Polygon(points=polygon_points)
        else:
            response.geometry = Polygon()

        # Compute center of mass of all relevant points and project to top surface
        center_xyz = np.mean(relevant_points, axis=0)
        center_xy = center_xyz[:2]
        center_z = np.max(z_values)  # project to top surface
        response.center = Point(x=float(center_xy[0]), y=float(center_xy[1]), z=float(center_z))

        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GeometryExtractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()