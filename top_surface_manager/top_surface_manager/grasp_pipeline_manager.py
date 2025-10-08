import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Polygon, PolygonStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from top_surface_interfaces.srv import (
    DetectObjects,
    ExtractObjectGeometry,
    SegmentObjects,
)


@dataclass
class Coordinate:
    x: float
    y: float

    def __sub__(self, other: "Coordinate") -> "Coordinate":
        return Coordinate(self.x - other.x, self.y - other.y)


class GraspPipelineManager(Node):
    def __init__(self) -> None:
        super().__init__('top_surface_manager')

        self.cb_group = ReentrantCallbackGroup()

        # publishers for visualization
        self.poly_pub = self.create_publisher(PolygonStamped, '/top_surface_polygon', 10)
        self.center_pub = self.create_publisher(PointStamped, '/top_surface_center', 10)


        # Subscribe to RGB image topic
        self.rgb_sub = self.create_subscription(
            Image, '/realsense/image_raw', self.rgb_callback, 10, callback_group=self.cb_group
        )

        # Subscribe to depth data topic
        self.depth_sub = self.create_subscription(
            Image, '/realsense/depth/image_raw', self.depth_callback, 10, callback_group=self.cb_group
        )

        # Subscribe to point cloud topic
        self.pc_sub = self.create_subscription(
            PointCloud2, '/realsense/points', self.pc_callback, 10, callback_group=self.cb_group
        )

        # Subscribe to camera info topic
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/realsense/camera_info', self.camera_info_callback, 10, callback_group=self.cb_group
        )

        # Service to trigger grasp pipeline
        self.get_object_grasps: Service = self.create_service(
            Trigger, 'start_grasp_pipeline', self.grasp_pipeline, callback_group=self.cb_group
        )

        # Service clients (attach to same reentrant group)
        self.object_detect_client: Client = self.create_client(
            DetectObjects, 'detect_objects', callback_group=self.cb_group
        )
        self.segment_object_client: Client = self.create_client(
            SegmentObjects, 'segment_objects', callback_group=self.cb_group
        )
        self.shape_from_pc_client: Client = self.create_client(
            ExtractObjectGeometry, 'extract_object_geometry', callback_group=self.cb_group
        )

        # Internal state
        self.rgb_image: Optional[Image] = None
        self.depth_image: Optional[Image] = None
        self.point_cloud: Optional[PointCloud2] = None
        self.camera_info: Optional[CameraInfo] = None
        self.data_frozen: bool = False

        # cv bridge
        self.br = CvBridge()

    # TODO: Implement callback functions for subscribers
    def rgb_callback(self, msg: Image) -> None:
        if not self.data_frozen:
            self.rgb_image = msg

    def depth_callback(self, msg: Image) -> None:
        if not self.data_frozen:
            self.depth_image = msg

    def pc_callback(self, msg: PointCloud2) -> None:
        if not self.data_frozen:
            self.point_cloud = msg
    
    def camera_info_callback(self, msg: CameraInfo) -> None:
        if not self.data_frozen:
            self.camera_info = msg

    def grasp_pipeline(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.data_frozen = True  # Freeze data during processing
        if self.rgb_image is None or self.depth_image is None or self.point_cloud is None:
            response.success = False
            missing = []
            if self.rgb_image is None:
                missing.append("RGB image")
            if self.depth_image is None:
                missing.append("Depth image")
            if self.point_cloud is None:
                missing.append("Point cloud")
            response.message = f"Data not available: {', '.join(missing)}."
            return response
        self.get_logger().info("Data available starting grasp pipline...")
        
        # # Call object detector service
        # while not self.object_detect_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for object detection service...')
        # detect_req = DetectObjects.Request()
        # detect_req.image = self.rgb_image
        # future = self.object_detect_client.call_async(detect_req)
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is None:
        #     self.get_logger().error('Object detection service call failed')
        #     response.success = False
        #     response.message = "Object detection service call failed."
        #     self.data_frozen = False
        #     return response
        
        # # Detections
        # detections_future = future.result()
        # assert isinstance(detections_future, DetectObjects.Response)
        # detections: Detection2DArray = detections_future.detections
        
        # # Call segment object service
        # while not self.segment_object_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for object segmentation service...')
        # segment_req = SegmentObjects.Request()
        # segment_req.image = self.rgb_image
        # segment_req.detections = detections
        # future = self.segment_object_client.call_async(segment_req)
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is None:
        #     self.get_logger().error('Segmentation service call failed')
        #     response.success = False
        #     response.message = "Segmentation service call failed."
        #     self.data_frozen = False
        #     return response
        
        # # Segmented masks
        # segmented_future = future.result()
        # assert isinstance(segmented_future, SegmentObjects.Response)
        # segmented_detections: Sequence[Image] = cast(Sequence[Image], segmented_future.masks)
        # if len(segmented_detections) == 0:
        #     self.get_logger().error('Segmentation returned no masks')
        #     response.success = False
        #     response.message = "Segmentation returned no masks."
        #     self.data_frozen = False
        #     return response

        # # Call shape from point cloud service
        # for obj_mask in segmented_detections:
        #     if obj_mask.data == []:
        #         self.get_logger().error('Segmentation returned empty mask')
        #         response.success = False
        #         response.message = "Segmentation returned empty mask."
        #         self.data_frozen = False
        #         return response

        # For testing, use some example segmented detections
        segmented_detections = []

        def create_mask(shape, rgb_image):
            # Get image size from the RGB image
            cv_img = self.br.imgmsg_to_cv2(rgb_image, desired_encoding="bgr8")
            height, width = cv_img.shape[:2]
            mask = np.zeros((height, width), dtype=np.uint8)
            center = (width // 2, height // 2)
            if shape == "square":
                side = min(width, height) // 4
                top_left = (center[0] - side // 2, center[1] - side // 2)
                bottom_right = (center[0] + side // 2, center[1] + side // 2)
                cv2.rectangle(mask, top_left, bottom_right, 255, -1)
            elif shape == "rectangle":
                w, h = width // 3, height // 6
                top_left = (center[0] - w // 2, center[1] - h // 2)
                bottom_right = (center[0] + w // 2, center[1] + h // 2)
                cv2.rectangle(mask, top_left, bottom_right, 255, -1)
            elif shape == "triangle":
                pts = np.array([
                    [center[0], center[1] - height // 6],
                    [center[0] - width // 8, center[1] + height // 6],
                    [center[0] + width // 8, center[1] + height // 6]
                ], np.int32)
                cv2.fillPoly(mask, [pts], 255)
            elif shape == "circle":
                radius = min(width, height) // 8
                cv2.circle(mask, center, radius, 255, -1)
            elif shape == "ellipse":
                axes = (width // 6, height // 12)
                cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
            return mask

        shapes = ["square", "rectangle", "triangle", "circle", "ellipse"]
        for shape in shapes:
            mask = create_mask(shape, self.rgb_image)
            img_msg = self.br.cv2_to_imgmsg(mask, encoding="mono8")
            segmented_detections.append(img_msg)
        self.get_logger().info("Made shapes")
        
        grasps: dict[int, list] = {}
        index=-1
        for obj_mask in segmented_detections:
            index += 1
            self.get_logger().info("Looking at obj mask")
            while not self.shape_from_pc_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for shape from point cloud service...')
            self.get_logger().info("Top surface service available")
            shape_req = ExtractObjectGeometry.Request()
            shape_req.cloud = self.point_cloud
            shape_req.mask = obj_mask
            future = self.shape_from_pc_client.call_async(shape_req)
            start_time = time.time()
            timeout_sec = 10.0
            while not future.done():
                # Sleep briefly to yield thread; callbacks executing in other threads will set the future.
                time.sleep(0.01)
                if (time.time() - start_time) > timeout_sec:
                    self.get_logger().error('Geometry service call timed out')
                    response.success = False
                    response.message = "Shape from point cloud service call timed out."
                    self.data_frozen = False
                    return response
            shape_future = future.result()
            if shape_future is None:
                self.get_logger().error('Shape from point cloud service returned None')
                response.success = False
                response.message = "Shape from point cloud service returned None."
                self.data_frozen = False
                return response
            self.get_logger().info("Got shape from point cloud")
            assert isinstance(shape_future, ExtractObjectGeometry.Response)
            # Geometry of top surface
            geometry: Polygon = shape_future.geometry
            point_center: Point = shape_future.center
            center: Coordinate = Coordinate(point_center.x, point_center.y)

            self.get_logger().info(f"Found obj information {geometry}, {point_center}, {center}")
            # Publish for visualization

            assert isinstance(self.point_cloud, PointCloud2)
            points = list(point_cloud2.read_points(self.point_cloud, field_names=["x", "y", "z"], skip_nans=True))
            top_z = 0.0
            if points:
                top_z = max([p[2] for p in points])

            # Create PolygonStamped message
            poly_msg = PolygonStamped()
            poly_msg.header.frame_id = self.point_cloud.header.frame_id if self.point_cloud else 'map'
            poly_msg.header.stamp = self.get_clock().now().to_msg()
            poly_msg.polygon = geometry
            # Set all polygon points' z to top_z
            for pt in poly_msg.polygon.points:
                pt.z = float(top_z)
            self.poly_pub.publish(poly_msg)

            # Create PointStamped message
            center_msg = PointStamped()
            center_msg.header.frame_id = self.point_cloud.header.frame_id if self.point_cloud else 'map'
            center_msg.header.stamp = self.get_clock().now().to_msg()
            center_msg.point = point_center
            center_msg.point.z = float(top_z) if points else center_msg.point.z
            self.center_pub.publish(center_msg)



            if center is None and geometry is None: 
                self.get_logger().error('No object center found for grasp matrix generation')
                response.success = False
                response.message = "No object center found for grasp matrix generation."
                self.data_frozen = False
                return response
            
            best_min_sv = -np.inf
            best_volume = -np.inf
            best_isotropy = -np.inf
            best_grasp = None
            
            contact_pair_locations = self.get_all_possible_grasps(geometry, center)
            for contact_pair in contact_pair_locations:
                # Generate grasp matrix
                G = self.generate_grasp_matrix(obj_center=center, contact_locations=contact_pair) 
                # Evaluate grasp matrix
                min_sv, volume, isotopy = self.evaluate_grasp_quality(G)
                # Choose the grasp with the highest minimum singular value, then volume, then isotropy
                if (min_sv > best_min_sv) or (
                    min_sv == best_min_sv and volume > best_volume
                ) or (
                    min_sv == best_min_sv and volume == best_volume and isotopy > best_isotropy
                ):
                    best_min_sv = min_sv
                    best_volume = volume
                    best_isotropy = isotopy
                    best_grasp = contact_pair
            
            grasps[index] = [geometry, center, best_grasp, best_min_sv, best_volume, best_grasp]

            # debug by showing cv2 image of polygon, center, and grasp points
            # Convert mask to OpenCV image
            cv_img_color = self.br.imgmsg_to_cv2(self.rgb_image, desired_encoding="bgr8")
            mask_img = self.br.imgmsg_to_cv2(obj_mask, desired_encoding="mono8")

            # Overlay mask onto image (make mask area semi-transparent red)
            mask_color = np.zeros_like(cv_img_color)
            mask_color[:, :, 2] = 255  # Red channel
            alpha = 0.4
            mask_bool = mask_img > 0
            cv_img_color[mask_bool] = cv2.addWeighted(cv_img_color[mask_bool], 1 - alpha, mask_color[mask_bool], alpha, 0)

            # Draw polygon
            if geometry.points:
                pts = np.array([[int(p.x), int(p.y)] for p in geometry.points], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(cv_img_color, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw center
            center_xy = (int(center.x), int(center.y))
            cv2.circle(cv_img_color, center_xy, radius=5, color=(0, 0, 255), thickness=-1)

            # Draw grasp locations (relative to center)
            if best_grasp is not None:
                grasp1 = (int(center.x + best_grasp[0].x), int(center.y + best_grasp[0].y))
                grasp2 = (int(center.x + best_grasp[1].x), int(center.y + best_grasp[1].y))
                cv2.circle(cv_img_color, grasp1, radius=5, color=(255, 0, 0), thickness=-1)
                cv2.circle(cv_img_color, grasp2, radius=5, color=(255, 0, 0), thickness=-1)
                cv2.line(cv_img_color, grasp1, grasp2, color=(255, 0, 0), thickness=2)

            # Show image for debugging
            debug_img_path = f"/tmp/grasp_debug_{index}.png"
            cv2.imwrite(debug_img_path, cv_img_color)
            self.get_logger().info(f"Saved debug image to {debug_img_path}")


        response.success = True
        response.message = "Grasp pipeline executed."
        return response

    # TODO: Implement getting all contact pairs for a parallel gripper on the geometry
    def get_all_possible_grasps(self, geometry: Polygon, center: Coordinate) -> list[tuple[Coordinate, Coordinate]]:
        # For a parallel gripper, return two contact points on opposite sides of the polygon
        # TODO: THIS IS JUST A PLACEHOLDER TO GET A CONTACT FOR TESTS
        if not geometry.points or len(geometry.points) < 2:
            return []
        points_list = list(geometry.points)
        p1 = points_list[0]
        p2 = points_list[len(points_list) // 2]
        contact1 = Coordinate(p1.x - center.x, p1.y - center.y)
        contact2 = Coordinate(p2.x - center.x, p2.y - center.y)
        return [(contact1, contact2)]
    
    def generate_grasp_matrix(self, obj_center: Coordinate, contact_locations: tuple[Coordinate, Coordinate]) -> np.ndarray:
        obj_xy = obj_center
        contact_xy = contact_locations
        num_contacts = len(contact_locations)
        G = np.zeros((3, 2 * num_contacts))

        for i in range(num_contacts):
            r = contact_xy[i] - obj_xy  # (2,)
            # hard contact can only apply forces in the plane with no moment because there is no friction
            # Each contact wrench: [f_x, f_y]
            # The mapping for each contact:
            # [F_x]   [1 0] [f_x]
            # [F_y] = [0 1] [f_y]
            Gi = np.array([[1, 0], [0, 1], [-r.y, r.x]])  # shape (3,2)
            G[:, 2 * i : 2 * i + 2] = Gi

        return G


    # TODO: Implement grasp quality evaluation
    def evaluate_grasp_quality(self, G: np.ndarray) -> tuple[float, float, float]:
        U, S, Vh = np.linalg.svd(G)
        print("S is: ", S)
        min_sv: float = float(np.min(S))
        volume: float = float((4 / 3) * np.pi * np.prod(S[:3]))
        isotropy: float = float(np.min(S) / np.max(S)) if np.max(S) > 0 else 0.0

        return min_sv, volume, isotropy

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GraspPipelineManager()
    from rclpy.executors import MultiThreadedExecutor
    # Use multiple threads so service calls made inside callbacks can complete.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()