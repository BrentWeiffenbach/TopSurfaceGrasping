from dataclasses import dataclass
from typing import Optional, Sequence, cast

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Polygon, PolygonStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from top_surface_interfaces.srv import (
    ExtractObjectGeometry,
    SegmentObjects,
)
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2


@dataclass
class Coordinate:
    x: float
    y: float
    z: float = 0.0

    def __sub__(self, other: "Coordinate") -> "Coordinate":
        return Coordinate(self.x - other.x, self.y - other.y, self.z - other.z)


class GraspPipelineManager(Node):
    def __init__(self) -> None:
        super().__init__('top_surface_manager')

        self.cb_group = ReentrantCallbackGroup()
        
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
        self.segment_object_client: Client = self.create_client(
            SegmentObjects, 'segment_objects', callback_group=self.cb_group
        )
        self.shape_from_pc_client: Client = self.create_client(
            ExtractObjectGeometry, 'extract_object_geometry', callback_group=self.cb_group
        )

        # Polygon and center accumulation
        self.object_polygons = []  # List[Polygon]
        self.object_centers = []   # List[Coordinate]
        self.all_top_surface_points = PointCloud2()  # Accumulate all top surface points

        # Publishers for polygons (up to 4) - use PolygonStamped for proper frame info
        self.polygon_publishers = [
            self.create_publisher(PolygonStamped, f'object_polygon_{i+1}', 10)
            for i in range(4)
        ]

        # Publisher for all centers as MarkerArray
        self.centers_marker_pub = self.create_publisher(MarkerArray, 'object_centers_marker_array', 10)
        # Publisher for all top surface points
        self.top_surface_points_pub = self.create_publisher(PointCloud2, 'all_top_surface_points', 10)

        self.best_grasps = []  # List of best grasps per object
        self.grasp_markers_pub = self.create_publisher(MarkerArray, 'grasp_markers', 10)

        # Internal state
        self.rgb_image: Optional[Image] = None
        self.depth_image: Optional[Image] = None
        self.point_cloud: Optional[PointCloud2] = None
        self.camera_info: Optional[CameraInfo] = None
        self.data_frozen: bool = False

        # cv bridge
        self.br = CvBridge()

        # tf buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def rgb_callback(self, msg: Image) -> None:
        if not self.data_frozen:
            self.rgb_image = msg

    def depth_callback(self, msg: Image) -> None:
        if not self.data_frozen:
            self.depth_image = msg

    def sanitize_pointcloud(self, msg: PointCloud2) -> PointCloud2:
        # Extract all points as a list of tuples (slower but ensures consistency)
        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        
        # Create a new, minimal PointCloud2
        sanitized = pc2.create_cloud(
            msg.header,
            [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            points
        )
        return sanitized

    def pc_callback(self, msg: PointCloud2) -> None:
        if not self.data_frozen:
            try:
                msg = self.sanitize_pointcloud(msg)
                msg_time = Time.from_msg(msg.header.stamp)
                
                if self.tf_buffer.can_transform('world', msg.header.frame_id, msg_time, Duration(seconds=1)):
                    transform = self.tf_buffer.lookup_transform(
                        'world',
                        msg.header.frame_id,
                        msg_time,
                        timeout=Duration(seconds=1)
                    )
                    transformed_pc = tf2_sensor_msgs.do_transform_cloud(msg, transform)
                    self.point_cloud = transformed_pc
                else:
                    self.get_logger().warn(f"Transform from {msg.header.frame_id} to world not available, using original point cloud")
                    self.point_cloud = msg
            except Exception as e:
                self.get_logger().error(f"Failed to transform point cloud: {e}")
                self.point_cloud = msg
                    
    def camera_info_callback(self, msg: CameraInfo) -> None:
        if not self.data_frozen:
            self.camera_info = msg

    def grasp_pipeline(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.data_frozen = True  # Freeze data during processing

        # Clear previous results
        self.object_polygons = []
        self.object_centers = []

        try:
            if self.rgb_image is None or self.point_cloud is None:
                response.success = False
                missing = []
                if self.rgb_image is None:
                    missing.append("RGB image")
                if self.point_cloud is None:
                    missing.append("Point cloud")
                response.message = f"Data not available: {', '.join(missing)}."
                return response
            
            self.get_logger().info("Data available, starting pipeline...")
            
            # Call segment object service synchronously
            while not self.segment_object_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for object segmentation service...')
            
            segment_req = SegmentObjects.Request()
            segment_req.image = self.rgb_image
            segmented_result = self.segment_object_client.call(segment_req)
            
            if segmented_result is None:
                self.get_logger().error('Segmentation service call failed')
                response.success = False
                response.message = "Segmentation service call failed."
                return response
            
            # Get segmented masks
            segmented_detections: Sequence[Image] = cast(Sequence[Image], segmented_result.masks)
            if len(segmented_detections) == 0:
                self.get_logger().error('Segmentation returned no masks')
                response.success = False
                response.message = "Segmentation returned no masks."
                return response

            self.get_logger().info(f"Found {len(segmented_detections)} objects")
            
            # Process each detected object
            for index, obj_mask in enumerate(segmented_detections):
                self.get_logger().info(f"Processing object {index + 1}")
                
                # Call geometry extraction service synchronously
                while not self.shape_from_pc_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for geometry extraction service...')
                
                shape_req = ExtractObjectGeometry.Request()
                shape_req.cloud = self.point_cloud
                shape_req.mask = obj_mask
                self.get_logger().info("Calling geometry extraction service...")
                shape_result: ExtractObjectGeometry.Response = self.shape_from_pc_client.call(shape_req)
                
                if shape_result is None:
                    self.get_logger().error(f'Geometry extraction failed for object {index + 1}')
                    continue

                # Get geometry and center
                geometry: Polygon = shape_result.geometry
                point_center: Coordinate = Coordinate(shape_result.center.x, shape_result.center.y, shape_result.center.z)
                top_surface_points: PointCloud2 = shape_result.top_surface_points

                if not geometry.points:
                    self.get_logger().warn(f'No geometry found for object {index + 1}')
                    continue

                self.get_logger().info(f"Object {index + 1}: polygon with {len(geometry.points)} points, center at ({point_center.x:.3f}, {point_center.y:.3f}, {point_center.z:.3f})")

                # Accumulate polygons and centers
                self.object_polygons.append(geometry)
                self.object_centers.append(point_center)
                # Accumulate top surface points into a combined point cloud
                if not self.all_top_surface_points.data:
                    self.all_top_surface_points = top_surface_points
                else:
                    # Concatenate point cloud data
                    combined_points = list(pc2.read_points(self.all_top_surface_points, field_names=["x", "y", "z"], skip_nans=True)) + \
                                      list(pc2.read_points(top_surface_points, field_names=["x", "y", "z"], skip_nans=True))
                    self.all_top_surface_points = pc2.create_cloud(
                        top_surface_points.header,
                        [
                            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                        ],
                        combined_points
                    )

                # get grasp candidates
                contact_pairs = self.get_all_possible_grasps(geometry, point_center)
                if not contact_pairs:
                    self.get_logger().warn(f'No grasp candidates found for object {index + 1}')
                    continue
                best_contacts = None
                best_min_sv = -float('inf')
                best_volume = 0.0
                best_isotropy = 0.0
                for i, contacts in enumerate(contact_pairs):
                    G = self.generate_grasp_matrix(point_center, contacts)
                    min_sv, volume, isotropy = self.evaluate_grasp_quality(G)
                    self.get_logger().info(f"  Grasp {i + 1}: min_sv={min_sv:.4f}, volume={volume:.4f}, isotropy={isotropy:.4f}")
                    score = 0
                    if min_sv > best_min_sv:
                        score += 1
                    if volume > best_volume:
                        score += 1
                    if isotropy > best_isotropy:
                        score += 1
                    if score >= 2:
                        best_min_sv = min_sv
                        best_volume = volume
                        best_isotropy = isotropy
                        best_contacts = contacts
                if best_contacts:
                    self.get_logger().info(f"Object {index + 1}: Best grasp with min_sv={best_min_sv:.4f}, volume={best_volume:.4f}, isotropy={best_isotropy:.4f}")
                    # publish markers at contact points
                    self.best_grasps.append((point_center, best_contacts))
                else:
                    self.get_logger().warn(f'No valid grasps found for object {index + 1}')
                    continue

            # PUBLISH ALL OBJECT DATA AT END
            header = self.point_cloud.header
            header.frame_id = "world"
            
            # 1. Publish combined top surface points 
            try:
                if self.all_top_surface_points.data:
                    self.top_surface_points_pub.publish(self.all_top_surface_points)
                    self.get_logger().info("Published combined top surface point cloud")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish top surface points: {e}")
            
            # 2. Publish up to 4 polygons as PolygonStamped with proper header
            for i in range(min(4, len(self.object_polygons))):
                try:
                    polygon_msg = PolygonStamped()
                    polygon_msg.header = header
                    polygon_msg.polygon = self.object_polygons[i]
                    self.polygon_publishers[i].publish(polygon_msg)
                    
                    # Debug info - exactly like the service
                    ys = [p.y for p in self.object_polygons[i].points] 
                    center = self.object_centers[i]
                    self.get_logger().info("=== CURRENT OBJECT ===")
                    self.get_logger().info(f"Polygon: {len(self.object_polygons[i].points)} vertices at Y={ys[0]:.4f}")
                    self.get_logger().info(f"Center: ({center.x:.3f}, {center.y:.3f}, {center.z:.3f})")
                    self.get_logger().info("Published polygon and points")
                    self.get_logger().info("====================")
                except Exception as e:
                    self.get_logger().warn(f"Failed to publish polygon: {e}")
            
            # 3. Publish all centers as MarkerArray 
            try:
                marker_array = MarkerArray()
                marker_array.markers = []
                
                for i, center in enumerate(self.object_centers):
                    marker = Marker()
                    marker.header = header
                    marker.ns = "object_center"  # Same namespace as service
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = center.x
                    marker.pose.position.y = center.y
                    marker.pose.position.z = center.z
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.025  # 2.5 cm diameter sphere
                    marker.scale.y = 0.025
                    marker.scale.z = 0.025
                    marker.color.r = 1.0  # Red color 
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8  # Semi-transparent
                    marker_array.markers.append(marker)
                
                self.centers_marker_pub.publish(marker_array)
                self.get_logger().info("Published center markers")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish center markers: {e}")

            
            # 4. Publish all best grasps as MarkerArray
            try:
                grasp_marker_array = MarkerArray()
                grasp_marker_array.markers = []
                
                for i, (center, contacts) in enumerate(self.best_grasps):
                    for j, contact in enumerate(contacts):
                        marker = Marker()
                        marker.header = header
                        marker.ns = "grasp_contact"
                        marker.id = i * 10 + j  # Unique ID per contact
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = center.x + contact.x
                        marker.pose.position.y = center.y + contact.y
                        marker.pose.position.z = center.z + contact.z
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.03  # 3cm diameter sphere for contacts
                        marker.scale.y = 0.03
                        marker.scale.z = 0.03
                        marker.color.r = 1.0  # Purple color for contacts
                        marker.color.g = 0.0
                        marker.color.b = 1.0
                        marker.color.a = 0.8
                        grasp_marker_array.markers.append(marker)
                
                self.grasp_markers_pub.publish(grasp_marker_array)
                self.get_logger().info("Published grasp markers")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish grasp markers: {e}")

            response.success = True
            response.message = f"Pipeline completed successfully. Processed {len(segmented_detections)} objects."
            return response
            
        except Exception as e:
            self.get_logger().error(f"Exception in pipeline: {e}")
            response.success = False
            response.message = f"Pipeline failed: {str(e)}"
            return response
        finally:
            self.data_frozen = False

    # TODO: Implement getting all contact pairs for a parallel gripper on the geometry
    def get_all_possible_grasps(self, geometry: Polygon, center: Coordinate) -> list[tuple[Coordinate, Coordinate]]:
        # For a parallel gripper, return two contact points on opposite sides of the polygon
        # TODO: THIS IS JUST A PLACEHOLDER TO GET A CONTACT FOR TESTS
        if not geometry.points or len(geometry.points) < 2:
            return []
        points_list = list(geometry.points)
        p1 = points_list[0]
        p2 = points_list[len(points_list) // 2]
        contact1 = Coordinate(p1.x - center.x, p1.y - center.y, p1.z - center.z)
        contact2 = Coordinate(p2.x - center.x, p2.y - center.y, p2.z - center.z)
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