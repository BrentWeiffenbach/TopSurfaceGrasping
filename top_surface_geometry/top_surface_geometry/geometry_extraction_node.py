#!/usr/bin/env python3

import struct
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker
from top_surface_interfaces.srv import ExtractObjectGeometry
import sensor_msgs_py.point_cloud2 as pc2


def pointcloud2_to_xyz_array(cloud: PointCloud2) -> np.ndarray:
    # Assumes fields x, y, z are float32 and in order
    points = []
    point_step = cloud.point_step
    for i in range(cloud.width * cloud.height):
        offset = i * point_step
        x, y, z = struct.unpack_from("fff", cloud.data, offset)
        points.append([x, y, z])
    return np.array(points, dtype=np.float32)


def points_to_contour_polygon(
    points_2d: np.ndarray, resolution: float = 0.005
) -> np.ndarray:
    """
    Convert 2D points to polygon using OpenCV Canny edge detection and contour finding.
    This provides better boundary detection for curved objects compared to ConvexHull.
    """
    if points_2d.shape[0] < 3:
        print("Not enough points for contour detection.")
        return points_2d

    # Find the bounding box of the points
    min_vals = np.min(points_2d, axis=0)
    max_vals = np.max(points_2d, axis=0)

    # Create a grid with the specified resolution
    grid_width = (
        int(np.ceil((max_vals[0] - min_vals[0]) / resolution)) + 20
    )  # Add padding
    grid_height = int(np.ceil((max_vals[1] - min_vals[1]) / resolution)) + 20

    # Ensure minimum grid size
    grid_width = max(grid_width, 50)
    grid_height = max(grid_height, 50)

    # Create binary image
    image = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Map 3D points to 2D grid coordinates
    grid_points = np.round(
        ((points_2d - min_vals + resolution * 5) / resolution)
    ).astype(int)

    # Ensure points are within image bounds
    valid_mask = (
        (grid_points[:, 0] >= 0)
        & (grid_points[:, 0] < grid_width)
        & (grid_points[:, 1] >= 0)
        & (grid_points[:, 1] < grid_height)
    )
    valid_points = grid_points[valid_mask]

    if len(valid_points) == 0:
        print("No valid points after mapping to grid.")
        return points_2d

    # Set pixels where points exist
    image[valid_points[:, 1], valid_points[:, 0]] = 255

    # Apply morphological operations to fill gaps and create connected regions
    kernel = np.ones((3, 3), np.uint8)
    image = cv2.dilate(image, kernel, iterations=2)
    image = cv2.erode(image, kernel, iterations=1)

    # Apply Gaussian blur to smooth the edges
    image = cv2.GaussianBlur(image, (9, 9), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(image, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No contours found.")
        return points_2d

    # Get the largest contour (assuming it's the object boundary)
    largest_contour = max(contours, key=cv2.contourArea)

    # Simplify contour to reduce number of points while preserving shape
    epsilon = 0.007 * cv2.arcLength(largest_contour, True)
    simplified_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

    # Convert back to world coordinates
    contour_2d = simplified_contour.reshape(-1, 2).astype(np.float32)
    world_contour = (contour_2d * resolution) + (min_vals - resolution * 5)

    return world_contour


class GeometryExtractionNode(Node):
    def __init__(self) -> None:
        super().__init__("geometry_extraction_node")

        # Create service
        self.service: Service = self.create_service(
            ExtractObjectGeometry,
            "extract_object_geometry",
            self.extract_geometry_callback,
        )

        # Publishers for current object data
        self.top_points_publisher = self.create_publisher(
            PointCloud2, "top_surface_points", 10
        )
        self.center_publisher = self.create_publisher(Marker, "object_center", 10)
        self.polygon_publisher = self.create_publisher(PolygonStamped, "current_polygon", 10)

        # Constants
        self.Y_BAND = 0.01  # meters
        self.MIN_POINTS = 30  # minimum points required to attempt hull
        self.CONTOUR_RESOLUTION = 0.001  # Resolution for contour grid (meters)

        self.get_logger().info("Geometry extraction service ready")



    def extract_geometry_callback(
        self,
        request: ExtractObjectGeometry.Request,
        response: ExtractObjectGeometry.Response,
    ) -> ExtractObjectGeometry.Response:
        try:
            self.get_logger().info("Received geometry extraction request")
            mask = request.mask  # may be empty
            cloud = request.cloud
            assert isinstance(cloud, PointCloud2)

            # Convert PointCloud2 to numpy array
            points_xyz = pointcloud2_to_xyz_array(cloud)

            # Only use the masked points
            relevant_points = None
            if (
                isinstance(mask, Image)
                and mask.height > 0
                and mask.width > 0
                and len(mask.data) > 0
            ):
                try:
                    mask_np = np.frombuffer(mask.data, dtype=np.uint8).reshape(
                        mask.height, mask.width
                    )
                    relevant_indices = np.argwhere(mask_np > 0)
                    self.get_logger().info(
                        f"Mask provided with {len(relevant_indices)} positive pixels"
                    )
                    rel_pts = []
                    for y, x in relevant_indices:
                        idx = y * mask.width + x
                        if idx < points_xyz.shape[0]:
                            rel_pts.append(points_xyz[idx])
                    relevant_points = np.array(rel_pts, dtype=np.float32)
                except Exception as e:
                    self.get_logger().warn(f"Failed to use mask: {e}")
                    relevant_points = np.array([], dtype=np.float32)
            else:
                self.get_logger().info("Mask is empty, returning nothing")
                response.geometry = Polygon()
                response.center = Point()
                return response

            self.get_logger().info(f"Relevant points count: {relevant_points.shape[0]}")

            if relevant_points.size == 0:
                self.get_logger().warn("No points found in the masked region")
                response.geometry = Polygon()
                response.center = Point()
                return response


            # Select all points with no other points above them (for each x,z, pick min y)
            grid_res = self.CONTOUR_RESOLUTION if hasattr(self, 'CONTOUR_RESOLUTION') else 0.001
            xz = np.round(relevant_points[:, [0, 2]] / grid_res).astype(int)
            unique_xz, inverse_indices = np.unique(xz, axis=0, return_inverse=True)
            top_points = []
            for idx in range(len(unique_xz)):
                mask = inverse_indices == idx
                y_vals = relevant_points[mask, 1]
                min_idx = np.argmin(y_vals)
                top_points.append(relevant_points[mask][min_idx])
            top_points = np.array(top_points, dtype=np.float32)
            if top_points.shape[0] > 0:
                y_min_val = float(np.min(top_points[:, 1]))
                y_max_val = float(np.max(top_points[:, 1]))
                self.get_logger().info(f"Selected {top_points.shape[0]} top surface points (unique x,z), y range: min={y_min_val:.5f} max={y_max_val:.5f}")
            else:
                y_min_val = 0.0
                y_max_val = 0.0
                self.get_logger().info("No top surface points found.")

            if top_points.shape[0] >= 3:
                try:
                    # Project to Z-X plane (columns [2, 0])
                    points_2d = top_points[:, [2, 0]]  # Z, X coordinates
                    self.get_logger().info(f"2D points for hull: {points_2d.shape[0]} points")
                    self.get_logger().info(f"2D bounds: Z[{points_2d[:, 0].min():.3f}, {points_2d[:, 0].max():.3f}], X[{points_2d[:, 1].min():.3f}, {points_2d[:, 1].max():.3f}]")

                    # Use contour-based polygon extraction
                    contour_2d = points_to_contour_polygon(points_2d, resolution=self.CONTOUR_RESOLUTION)
                    self.get_logger().info(f"Contour polygon created with {contour_2d.shape[0]} vertices")

                    # Create 3D polygon points at the SAME Y level as top_surface_points
                    polygon_points = []
                    for pt in contour_2d:
                        z_coord = float(pt[0])  # Z coordinate
                        x_coord = float(pt[1])  # X coordinate
                        y_coord = float(y_min_val)  # Y at top surface level
                        polygon_points.append(Point32(x=x_coord, y=y_coord, z=z_coord))

                    response.geometry = Polygon(points=polygon_points)
                    self.get_logger().info(f"Created contour polygon with {len(polygon_points)} vertices at Y={y_min_val:.3f}")

                except Exception as e:
                    self.get_logger().error(f"Polygon Extraction failed: {e}")
                    response.geometry = Polygon()
            else:
                response.geometry = Polygon()
                self.get_logger().warn(f"Not enough points for Polygon Extraction: {top_points.shape[0]} < 3")

            # Compute center: use center of mass of all top surface points
            center = top_points.mean(axis=0)
            response.center = Point(
                x=float(center[0]), y=float(center[1]), z=float(center[2])
            )

            # PUBLISH CURRENT OBJECT DATA
            header = cloud.header
            header.frame_id = "world"
            
            # 1. Publish top surface points for this object
            try:
                if top_points.size > 0:
                    top_points_msg = pc2.create_cloud_xyz32(header, top_points.tolist())
                    response.top_surface_points = top_points_msg
                    self.top_points_publisher.publish(top_points_msg)
                    self.get_logger().info(f"Published top surface point cloud with {top_points.shape[0]} points")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish top surface points: {e}")
            
            # 2. Publish polygon for this object
            if len(response.geometry.points) > 0:
                try:
                    polygon_msg = PolygonStamped()
                    polygon_msg.header = header
                    polygon_msg.polygon = response.geometry
                    self.polygon_publisher.publish(polygon_msg)
                    
                    # Debug info
                    ys = [p.y for p in response.geometry.points] 
                    self.get_logger().info("=== CURRENT OBJECT ===")
                    self.get_logger().info(f"Polygon: {len(response.geometry.points)} vertices at Y={ys[0]:.4f}")
                    self.get_logger().info(f"Center: ({response.center.x:.3f}, {response.center.y:.3f}, {response.center.z:.3f})")
                    self.get_logger().info("Published polygon and points")
                    self.get_logger().info("====================")
                except Exception as e:
                    self.get_logger().warn(f"Failed to publish polygon: {e}")
            else:
                self.get_logger().warn("No polygon created - check ConvexHull generation")
            
            # 3. Publish center marker for this object
            try:
                marker = Marker()
                marker.header = header
                marker.ns = "object_center"
                marker.id = 0  # Single object, always ID 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = response.center
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.02  # 2cm diameter sphere
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.r = 1.0  # Red color
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8  # Semi-transparent
                
                self.center_publisher.publish(marker)
                self.get_logger().info("Published center marker")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish center marker: {e}")

            self.get_logger().info("Geometry extraction completed...")
            return response

        except Exception as e:
            self.get_logger().error(f"Exception in geometry extraction: {e}")
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


if __name__ == "__main__":
    main()
