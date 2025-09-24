#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point
from top_surface_interfaces.srv import ExtractObjectGeometry


class GeometryExtractionNode(Node):
    def __init__(self):
        super().__init__('geometry_extraction_node')
        
        # Create service
        self.service = self.create_service(
            ExtractObjectGeometry,
            'extract_object_geometry',
            self.extract_geometry_callback
        )
        
        self.get_logger().info('Geometry extraction service ready')
    
    def extract_geometry_callback(self, request, response):
        """
        TODO: Implement object geometry and center extraction from point cloud and mask
        - Take the input point cloud (request.cloud) and segmentation mask (request.mask)
        - Apply mask to filter relevant points from the point cloud
        - Extract the top surface of the object from the filtered point cloud
        - Compute 2D geometry (polygon) representing the top surface shape
        - Calculate the center of mass of the full object, and project upwards to the top surface
        - Return the 2D polygon geometry and 2D center of mass coordinate
        """
        
        # For now, return empty geometry and zero center
        response.geometry = Polygon()
        response.geometry.points = []  # Empty list of polygon points
        
        response.center = Point()
        response.center.x = 0.0
        response.center.y = 0.0
        response.center.z = 0.0  # Z=0 for 2D projection
        
        self.get_logger().info('Processed extract geometry request - returning empty geometry and zero center')
        return response


def main(args=None):
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