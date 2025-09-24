#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from top_surface_interfaces.srv import SegmentObjects


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Create service
        self.service = self.create_service(
            SegmentObjects,
            'segment_objects',
            self.segment_objects_callback
        )
        
        self.get_logger().info('Segmentation service ready')
    
    def segment_objects_callback(self, request, response):
        """
        TODO: Implement object segmentation from detection boxes and RGB image
        - Take the input RGB image (request.image) and detections (request.detections)
        - For each detection bounding box, create a segmentation mask
        - Use a segmentation algorithim to generate masks
        - Convert masks to Image messages
        - Return array of segmentation masks corresponding to each detection
        """
        
        # For now, return empty list of masks
        response.masks = []
        
        # Create empty masks for each detection (all zeros)
        for detection in request.detections.detections:
            empty_mask = Image()
            empty_mask.header = request.image.header
            empty_mask.height = request.image.height
            empty_mask.width = request.image.width
            empty_mask.encoding = "mono8"  # Single channel mask
            empty_mask.step = request.image.width
            empty_mask.data = [0] * (request.image.height * request.image.width)
            response.masks.append(empty_mask)
        
        self.get_logger().info(f'Processed segment objects request - returning {len(response.masks)} empty masks')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()