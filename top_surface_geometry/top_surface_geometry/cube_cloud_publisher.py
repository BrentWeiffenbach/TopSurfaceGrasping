import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
import numpy as np
import struct

def create_pointcloud2(points, frame_id="cube_frame"):
    msg = PointCloud2()
    msg.header.stamp = rclpy.time.Time().to_msg() # type: ignore
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b''.join([struct.pack('fff', *p) for p in points])
    return msg

class CubeCloudPublisher(Node):
    def __init__(self):
        super().__init__('cube_cloud_publisher')
        self.cloud_pub = self.create_publisher(PointCloud2, 'cube_cloud', 1)
        self.mask_pub = self.create_publisher(Image, 'cube_mask', 1)
        self.timer = self.create_timer(1.0, self.publish_cube)

    def publish_cube(self):
        width, height = 10, 10
        points = []
        for z in np.linspace(0, 1, 10):
            for y in np.linspace(0, 1, height):
                for x in np.linspace(0, 1, width):
                    points.append((x, y, z))
        cloud_msg = create_pointcloud2(points, frame_id='cube_frame')

        # Mask for half of the top surface (left half only)
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[:, :width//2] = 255  # Only mask the left half
        mask_msg = Image()
        mask_msg.height = height
        mask_msg.width = width
        mask_msg.encoding = 'mono8'
        mask_msg.data = mask.flatten().tolist()
        mask_msg.step = width

        self.cloud_pub.publish(cloud_msg)
        self.mask_pub.publish(mask_msg)
        self.get_logger().info('Published cube cloud and mask')

def main(args=None):
    rclpy.init(args=args)
    node = CubeCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()