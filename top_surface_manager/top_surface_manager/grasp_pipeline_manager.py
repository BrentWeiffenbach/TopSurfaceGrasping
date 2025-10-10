import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple, cast

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.service import Service
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_srvs.srv import Trigger
from top_surface_interfaces.srv import (
    ExtractObjectGeometry,
    SegmentObjects,
)
from visualization_msgs.msg import Marker, MarkerArray


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
        self.best_grasps = []

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
                contact_pairs = self.get_all_possible_grasps(geometry, point_center, thetaDelta=math.pi/16)
                if not contact_pairs:
                    self.get_logger().warn(f'No grasp candidates found for object {index + 1}')
                    continue
                best_contacts = None
                best_min_sv = -float('inf')
                best_volume = 0.0
                best_isotropy = 0.0
                best_score = -float('inf')
                for i, contacts in enumerate(contact_pairs):
                    G = self.generate_grasp_matrix(point_center, contacts)
                    min_sv, volume, isotropy = self.evaluate_grasp_quality(G)
                    # Distance of contacts to center of mass (should be not too close)
                    dist1 = math.hypot(contacts[0].x, contacts[0].z)
                    dist2 = math.hypot(contacts[1].x, contacts[1].z)
                    avg_dist = (dist1 + dist2) / 2.0
                    self.get_logger().info(
                        f"  Grasp {i + 1}: min_sv={min_sv:.4f}, volume={volume:.4f}, isotropy={isotropy:.4f}, avg_dist={avg_dist:.4f}"
                    )
                    # Weighted scoring
                    score = (
                        0.6 * min_sv +
                        0.1 * volume +
                        0.1 * isotropy -
                        0.2 * avg_dist 
                    )
                    if score > best_score:  # Use score as main criterion
                        best_score = score
                        best_min_sv = min_sv
                        best_volume = volume
                        best_isotropy = isotropy
                        best_contacts = contacts
                if best_contacts is not None:
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

    # all possible grasping functions
    def getIntersectingPoint(self, 
                             rayPoints: Tuple[Point32, Point32], 
                             sidePoints: Tuple[Point32, Point32]) -> Optional[Point32]:
        (pt, centroid) = rayPoints
        (curr, next) = sidePoints
        # NOTE: Grasp planning now uses XZ plane (z replaces prior y role). Keep logs consistent.
        # print(f'XZ intersection between {((pt.x, pt.z), (centroid.x, centroid.z)), ((curr.x, curr.z), (next.x, next.z))}')

        if pt == curr:  # skip identical start
            # print('point == curr')
            return None
        # Treat plane coords as (x, z)
        a_ray = centroid.z - pt.z
        b_ray = pt.x - centroid.x
        c_ray = a_ray * pt.x + b_ray * pt.z

        a_side = next.z - curr.z
        b_side = curr.x - next.x
        c_side = a_side * curr.x + b_side * curr.z

        ## find intersection
        d = a_ray*b_side - a_side*b_ray
        if abs(d) < 1e-9: 
            # print('lines are parallel')
            return None
        else: 
            x = (b_side*c_ray - b_ray*c_side)/d
            y = (a_ray*c_side - a_side*c_ray)/d

        if x >= min(curr.x, next.x) and x <= max(curr.x, next.x) and y >= min(curr.z, next.z) and y <= max(curr.z, next.z):
            intersection = Point32(x=x, y=centroid.y, z=y)  # keep world y from centroid (up axis), store plane z in point.z
            return intersection
        else: 
            # print('does not fit in intersection')
            return None
        

    def getOpposite(self, currPt, centroid, geometry):
        """
        return the opposite point from the current point
        """
        points_list = list(geometry.points)
        n = len(points_list)
        if n < 3:
            self.get_logger().warn("Degenerate polygon (<3 vertices) in getOpposite")
            return None

        intersectingPoints: list[Point32] = []
        for i in range(n):
            curr = points_list[i]
            nxt = points_list[(i + 1) % n]
            # skip edges incident to current point
            if currPt == curr or currPt == nxt:
                continue
            point = self.getIntersectingPoint(rayPoints=(currPt, centroid), sidePoints=(curr, nxt))
            if point is not None:
                intersectingPoints.append(point)

        if not intersectingPoints:
            return None
        dist_pt_to_centroid = math.hypot(centroid.x - currPt.x, centroid.z - currPt.z)
        farthest_intersection: Optional[Point32] = None
        max_dist = 0.0
        for intersection in intersectingPoints:
            dist_pt_to_intersection = math.hypot(intersection.x - currPt.x, intersection.z - currPt.z)
            # Only consider intersections beyond centroid distance to ensure opposite side
            if dist_pt_to_intersection > dist_pt_to_centroid and dist_pt_to_intersection > max_dist:
                max_dist = dist_pt_to_intersection
                farthest_intersection = intersection
        return farthest_intersection
        
    def getNextPoint(
        self,
        point: Point32,
        centroid: Coordinate,
        geometry: Polygon,
        thetaDelta: float
    ) -> Optional[Point32]:
        """
        point = current point
        centroid: center of polygon
        geometry: 
        thetaDelta: increments of theta to change by in radians 
        """
        points_list = list(geometry.points)
        n = len(points_list)
        if n < 3:
            self.get_logger().warn("Degenerate polygon (<3 vertices) in getNextPoint")
            return None

        # Operate in XZ plane; handle near-vertical line in x dimension
        if abs(point.x - centroid.x) < 0.00001:  # vertical in plane
            if point.z > centroid.z:
                newTheta = math.pi/2 + thetaDelta
            else:
                newTheta = 3 * math.pi/2 + thetaDelta
        else: 
            m_old = (point.z - centroid.z) / (point.x - centroid.x)
            newTheta = math.atan(m_old) + thetaDelta
            
        m_new = math.tan(newTheta)
        b_new = centroid.z - m_new * centroid.x

        if m_new > 100000000000: # vertical line through centroid
            dummyPoint = Point32(x=centroid.x, y=centroid.y, z=centroid.z + 10)
        else: 
            x_dummy = centroid.x + 10
            z_dummy = m_new * x_dummy + b_new
            # print(f'm: {m_new}\tb: {b_new}\tx_dummy: {x_dummy}\tz_dummy: {z_dummy}')
            dummyPoint = Point32(x=x_dummy, y=centroid.y, z=z_dummy)

        intersectingPoints: list[Point32] = []
        centroid_point32 = Point32(x=centroid.x, y=centroid.y, z=centroid.z)
        for i in range(n):
            curr = points_list[i]
            nxt = points_list[(i + 1) % n]
            ip = self.getIntersectingPoint(rayPoints=(dummyPoint, centroid_point32), sidePoints=(curr, nxt))
            if ip is not None:
                intersectingPoints.append(ip)
        
        min_diff = math.inf
        # print(f'intersection points: {intersectingPoints}')
        if not intersectingPoints:
            return None
        anglePt = math.atan2(point.z - centroid.z, point.x - centroid.x)
        closest_point: Optional[Point32] = None
        for intersection in intersectingPoints:
            angleInter = math.atan2(intersection.z - centroid.z, intersection.x - centroid.x)
            diff = (angleInter - anglePt) % (2 * math.pi)  # CCW difference
            delta_error = abs(diff - thetaDelta)
            if delta_error < min_diff:
                min_diff = delta_error
                closest_point = intersection
        return closest_point

    def get_all_possible_grasps(
        self,
        geometry: Polygon,
        centroid: Coordinate,
        thetaDelta: float
    ) -> list[tuple[Coordinate, Coordinate]]:
        """Generate grasp candidates by radial ray casting (XZ plane) similar to grasp_proto logic.

        For each angle theta we cast a ray from centroid in direction (cos theta, sin theta) in XZ,
        find first boundary intersection p(theta) and its opposite p(theta+pi), forming a grasp pair.
        """
        points_list = list(geometry.points)
        n = len(points_list)
        if n < 3:
            self.get_logger().warn("Degenerate polygon (<3 vertices) in get_all_possible_grasps")
            return []

        # Precompute edges (wrap)
        edges = [(points_list[i], points_list[(i + 1) % n]) for i in range(n)]

        def ray_edge_intersection(theta: float, p0x: float, p0z: float) -> Optional[Point32]:
            # Ray origin p0, direction (dx,dz)
            dx = math.cos(theta)
            dz = math.sin(theta)
            best_t = math.inf
            hit: Optional[Point32] = None
            for a, b in edges:
                # Edge in param form: a + u*(b-a), ray: p0 + t*d, need 2x2 solve in XZ
                ex = b.x - a.x
                ez = b.z - a.z
                denom = dx * (-ez) - dz * (-ex)  # determinant of [[dx, -ex],[dz,-ez]] with rearrangement
                if abs(denom) < 1e-9:
                    continue
                # Solve using Cramer's rule for t,u from:
                # p0x + dx*t = a.x + ex*u
                # p0z + dz*t = a.z + ez*u
                rx = a.x - p0x
                rz = a.z - p0z
                t = (rx * (-ez) - rz * (-ex)) / denom
                u = (dx * rz - dz * rx) / denom
                if t > 1e-6 and 0.0 <= u <= 1.0:  # forward ray, on segment
                    if t < best_t:
                        best_t = t
                        hit = Point32(x=p0x + dx * t, y=centroid.y, z=p0z + dz * t)
            return hit

        grasp_pairs: list[tuple[Point32, Point32]] = []
        theta = 0.0
        two_pi = 2 * math.pi
        used_primary: list[Point32] = []
        while theta < two_pi - 1e-6:
            p1 = ray_edge_intersection(theta, centroid.x, centroid.z)
            p2 = ray_edge_intersection(theta + math.pi, centroid.x, centroid.z)
            if p1 and p2:
                grasp_pairs.append((p1, p2))
                used_primary.append(p1)
            theta += thetaDelta

        # Deduplicate (order invariant)
        unique: list[tuple[Point32, Point32]] = []
        for a, b in grasp_pairs:
            dup = False
            for c, d in unique:
                same = math.dist((a.x, a.z), (c.x, c.z)) < 1e-3 and math.dist((b.x, b.z), (d.x, d.z)) < 1e-3
                rev = math.dist((a.x, a.z), (d.x, d.z)) < 1e-3 and math.dist((b.x, b.z), (c.x, c.z)) < 1e-3
                if same or rev:
                    dup = True
                    break
            if not dup:
                unique.append((a, b))

        self.get_logger().info(f"Calculated {len(unique)} unique grasp candidate(s)")
        for a, b in unique:
            self.get_logger().debug(f" grasp pair XZ radial: ({a.x:.4f},{a.z:.4f}) <-> ({b.x:.4f},{b.z:.4f})")

        result: list[tuple[Coordinate, Coordinate]] = []
        for a, b in unique:
            c1 = Coordinate(a.x - centroid.x, 0.0, a.z - centroid.z)
            c2 = Coordinate(b.x - centroid.x, 0.0, b.z - centroid.z)
            result.append((c1, c2))
        return result
    
    def generate_grasp_matrix(self, obj_center: Coordinate, contact_locations: tuple[Coordinate, Coordinate]) -> np.ndarray:
        """Generate planar grasp matrix for contacts in XZ plane (torque about Y).

        contact_locations are already relative to the centroid (obj_center). We
        therefore ignore obj_center for position difference to avoid double subtraction.
        Wrench basis: [F_x, F_z, tau_y]. Each contact provides two force components.
        Tau_y = r_x * f_z - r_z * f_x (right-hand rule with Y as normal).
        """
        num_contacts = len(contact_locations)
        G = np.zeros((3, 2 * num_contacts))
        for i in range(num_contacts):
            r = contact_locations[i]  # already relative
            Gi = np.array([
                [1, 0],          # F_x
                [0, 1],          # F_z
                [-r.z, r.x],     # tau_y
            ])
            G[:, 2 * i : 2 * i + 2] = Gi
        return G


    def evaluate_grasp_quality(self, G: np.ndarray) -> tuple[float, float, float]:
        U, S, Vh = np.linalg.svd(G)
        # Log singular values in a safe string form
        self.get_logger().debug(f"Singular values: {S.tolist()}")
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