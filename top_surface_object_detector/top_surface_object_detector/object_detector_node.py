#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from vision_msgs.msg import Detection2DArray
from top_surface_interfaces.srv import DetectObjects


class ObjectDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('object_detector_node')
        
        # Create service
        self.service: Service = self.create_service(
            DetectObjects,
            'detect_objects',
            self.detect_objects_callback
        )
        
        self.get_logger().info('Object detector service ready')
    
    def detect_objects_callback(
        self, 
        request: DetectObjects.Request, 
        response: DetectObjects.Response
    ) -> DetectObjects.Response:
        """
        TODO: Implement object detection on RGB image
        - Take the input RGB image (request.image)
        - Run object detection algorithm
        - Extract bounding boxes for detected objects
        - Convert to Detection2DArray format
        - Return populated Detection2DArray
        """
        
        # For now, return empty Detection2DArray
        response.detections = Detection2DArray()
        response.detections.header = request.image.header
        response.detections.detections = []  # Empty list of detections
        
        self.get_logger().info('Processed detect objects request - returning empty detections')
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()