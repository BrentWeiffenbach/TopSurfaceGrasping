#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import Image
from top_surface_interfaces.srv import SegmentObjects
from cv_bridge import CvBridge
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

        self.all_mask_publisher = self.create_publisher(Image, 'all_mask', 10)

        self.br = CvBridge()
        
        self.get_logger().info('Segmentation service ready')
    

    def backgroundMask(
        self, 
        img: np.ndarray, 
        color: np.ndarray = np.array([15, 148, 114]), 
        h_thresh: int = 10, 
        s_thresh: int = 75, 
        v_thresh: int = 30
    ) -> np.ndarray:
        """
        Returns masks of objects by differentiating from background colors
        img: img
        color: background color, hsv
        """
        hsvImg: np.ndarray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower: np.ndarray = np.array([
            max(color[0] - h_thresh, 0),
            max(color[1] - s_thresh, 0),
            max(color[2] - v_thresh, 0)
        ])
        upper: np.ndarray = np.array([
            min(color[0] + h_thresh, 179),
            min(color[1] + s_thresh, 255),
            min(color[2] + v_thresh, 255)
        ])

        bg_mask: np.ndarray = cv2.inRange(hsvImg, lower, upper)
        obj_mask: np.ndarray = cv2.bitwise_not(bg_mask)  # everything not background

        return obj_mask

    def getObjectMasks(
        self,
        img: np.ndarray,
        min_area: int = 1000,
        max_area: Optional[int] = None
    ) -> tuple[list[np.ndarray], np.ndarray]:
        """
        takes in a single image and returns a list of binary masks
        img: single frame
        min_area: used to exclude contours with a small amount of area, only returns valid objects
        max_area: not currently used but can be used to exclude contours that are large
        """
        masks: list[np.ndarray] = []

        tableMask: np.ndarray = self.backgroundMask(img, color=np.array([15, 148, 114]), h_thresh=10, s_thresh=75, v_thresh=30)
        skyMask: np.ndarray = self.backgroundMask(img, color=np.array([0, 0, 56]), h_thresh=3, s_thresh=5, v_thresh=5)
        combinedMask: np.ndarray = cv2.bitwise_and(tableMask, skyMask)

        # alternative to contours
        # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(combinedMask)
        # for i in range(1, num_labels):
        #     mask_i = np.uint8(labels==i) *255
        #     masks.append(mask_i)

        contours, _ = cv2.findContours(combinedMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area: float = cv2.contourArea(contour)
            if area < min_area:
                continue
            if max_area is not None and area > max_area:
                continue
            obj_mask: np.ndarray = np.zeros_like(combinedMask)
            cv2.drawContours(obj_mask, [contour], -1, 255, -1)
            masks.append(obj_mask)
        return masks, combinedMask


    def segment_objects_callback(
        self,
        request: SegmentObjects.Request,
        response: SegmentObjects.Response
    ) -> SegmentObjects.Response:
        """
        Implements object segmentation from detection boxes and RGB image
        - Converts input ROS Image to OpenCV image
        - Generates masks
        - Converts masks to ROS Image messages
        - Publishes masks and returns them in response
        """
        img_msg: Image = request.image
        if img_msg is None or not img_msg.data:
            self.get_logger().error('Received empty image in segmentation request')
            return response

        # Convert ROS Image message to OpenCV image using CvBridge
        try:
            img_np: np.ndarray = self.br.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion failed: {e}')
            return response

        masks, combinedMask = self.getObjectMasks(img_np)

        all_masks_msg: Image = self.br.cv2_to_imgmsg(combinedMask, encoding='mono8')
        self.all_mask_publisher.publish(all_masks_msg)

        # Convert masks to ROS Image messages and publish
        response.masks = []
        for i, mask_np in enumerate(masks):
            mask_msg: Image = self.br.cv2_to_imgmsg(mask_np, encoding='mono8')
            response.masks.append(mask_msg)


        self.get_logger().info(f'Processed segment objects request - returning {len(response.masks)} masks')
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