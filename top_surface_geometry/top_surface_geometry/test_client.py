import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PolygonStamped, PointStamped
from top_surface_interfaces.srv import ExtractObjectGeometry
from sensor_msgs_py import point_cloud2

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(ExtractObjectGeometry, 'extract_object_geometry')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.cloud_sub = self.create_subscription(PointCloud2, 'cube_cloud', self.cloud_cb, 1)
        self.mask_sub = self.create_subscription(Image, 'cube_mask', self.mask_cb, 1)
        self.poly_pub = self.create_publisher(PolygonStamped, 'extracted_polygon', 1)
        self.center_pub = self.create_publisher(PointStamped, 'extracted_center', 1)
        self.cloud = None
        self.mask = None

    def cloud_cb(self, msg):
        self.cloud = msg
        self.try_call()

    def mask_cb(self, msg):
        self.mask = msg
        self.try_call()

    def try_call(self):
        if self.cloud and self.mask:
            req = ExtractObjectGeometry.Request()
            req.cloud = self.cloud
            req.mask = self.mask
            future = self.cli.call_async(req)
            future.add_done_callback(self.resp_cb)

    def resp_cb(self, future):
        resp = future.result()
        print('Polygon:', resp.geometry)
        print('Center:', resp.center)

        # Publish PolygonStamped at the top of the pointcloud
        # Extract z values from the pointcloud
        assert isinstance(self.cloud, PointCloud2)
        points = list(point_cloud2.read_points(self.cloud, field_names=["x", "y", "z"], skip_nans=True))
        top_z = 0.0
        if points:
            top_z = max([p[2] for p in points])

        # Create PolygonStamped message
        poly_msg = PolygonStamped()
        poly_msg.header.frame_id = self.cloud.header.frame_id if self.cloud else 'map'
        poly_msg.header.stamp = self.get_clock().now().to_msg()
        poly_msg.polygon = resp.geometry
        
        # Set all polygon points' z to top_z
        for pt in poly_msg.polygon.points:
            pt.z = float(top_z)
            
        self.poly_pub.publish(poly_msg)

        # Publish PointStamped
        center_msg = PointStamped()
        center_msg.header.frame_id = self.cloud.header.frame_id if self.cloud else 'map'
        center_msg.header.stamp = self.get_clock().now().to_msg()
        center_msg.point = resp.center
        center_msg.point.z = float(top_z) if points else center_msg.point.z
        self.center_pub.publish(center_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()