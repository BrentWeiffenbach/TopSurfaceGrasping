#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import Image
from top_surface_interfaces.srv import SegmentObjects
import cv2
import numpy as np


class SegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__('segmentation_node')
        
        # Create service
        self.service: Service = self.create_service(
            SegmentObjects,
            'segment_objects',
            self.segment_objects_callback
        )
        
        self.get_logger().info('Segmentation service ready')
    

    def backgroundMask(self, img, color=np.array([15, 148, 114]), h_thresh=10, s_thresh=75, v_thresh=30):
        """
        Returns masks of objects by differentiating from backgroud colors
        img: img
        color: background color, hsv
        """

        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([
            max(color[0] - h_thresh, 0),
            max(color[1] - s_thresh, 0),
            max(color[2] - v_thresh, 0)
        ])
        upper = np.array([
            min(color[0] + h_thresh, 179),
            min(color[1] + s_thresh, 255),
            min(color[2] + v_thresh, 255)
        ])

        bg_mask = cv2.inRange(hsvImg, lower, upper)
        obj_mask = cv2.bitwise_not(bg_mask)  # everything not background

        return obj_mask

    def getObjectMasks(self, img, min_area=1000, max_area=None):
        """
        takes in a single image and returns a list of binary masks
        img: single frame
        min_area: used to exclude contours with a small amount of area, only returns valid objects
        max_area: not currently used but can be used to exclude contours that are large
        """

        masks = []

        tableMask = self.backgroundMask(img, color=np.array([15, 148, 114]), h_thresh=10, s_thresh=75, v_thresh=30)
        tableResult = cv2.bitwise_and(img, img, mask=tableMask)

        skyMask = self.backgroundMask(img, color=np.array([0, 0, 56]), h_thresh=3, s_thresh=5, v_thresh=5)
        skyResults = cv2.bitwise_and(img, img, mask=skyMask)

        combinedMask = cv2.bitwise_and(tableMask, skyMask)

        ## alternative to countours
        # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(combinedMask)

        # for i in range(1, num_labels):
        #     mask_i = np.uint8(labels==i) *255
        #     masks.append(mask_i)

        contours, _ = cv2.findContours(combinedMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours: 
            area = cv2.contourArea(contour)

            if area < min_area:
                continue
            if max_area is not None and area > max_area:
                continue
        
            obj_mask = np.zeros_like(combinedMask)
            cv2.drawContours(obj_mask, [contour], -1, 255, -1)
            masks.append(obj_mask)
        
        return masks


    def segment_objects_callback(
        self, 
        request: SegmentObjects.Request, 
        response: SegmentObjects.Response
    ) -> SegmentObjects.Response:
        
        """
        TODO: Implement object segmentation from detection boxes and RGB image
        - Take the input RGB image (request.image) and detections (request.detections)
        - For each detection bounding box, create a segmentation mask
        - Use a segmentation algorithim to generate masks
        - Convert masks to Image messages
        - Return array of segmentation masks corresponding to each detection
        """
        img = request.image
        masks = self.getObjectMasks(img)
        
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


def main(args: Optional[list] = None) -> None:
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