from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.client import Client
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_srvs.srv import Trigger
from top_surface_interfaces.srv import DetectObjects, ExtractObjectGeometry, SegmentObjects

class GraspPipelineManager(Node):
    def __init__(self) -> None:
        super().__init__('top_surface_manager')

        # TODO: Subscribe to RGB image topic
        # self.rgb_sub = self.create_subscription(Image, 'rgb_image_topic', self.rgb_callback, 10)

        # TODO: Subscribe to depth data topic
        # self.depth_sub = self.create_subscription(Image, 'depth_image_topic', self.depth_callback, 10)

        # TODO: Subscribe to point cloud topic
        # self.pc_sub = self.create_subscription(PointCloud2, 'point_cloud_topic', self.pc_callback, 10)

        # TODO: Subscribe to camera info topic
        # self.camera_info_sub = self.create_subscription(CameraInfo, 'camera_info_topic', self.camera_info_callback, 10)

        self.get_object_grasps: Service = self.create_service(Trigger, 'start_grasp_pipeline', self.grasp_pipeline)

        self.object_detect_client: Client = self.create_client(DetectObjects, 'detect_objects')
        self.segment_object_client: Client = self.create_client(SegmentObjects, 'segment_objects')
        self.shape_from_pc_client: Client = self.create_client(ExtractObjectGeometry, 'extract_object_geometry')

        # Internal state
        self.rgb_image: Optional[Image] = None
        self.depth_image: Optional[Image] = None
        self.point_cloud: Optional[PointCloud2] = None
        self.camera_info: Optional[CameraInfo] = None
        self.data_frozen: bool = False

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
            response.message = "Data not available."
            return response
        
        # TODO: Call object detector service
        # TODO: Call segment object service
        # TODO: Call shape from point cloud service

        # TODO: Generate grasp matrix
        self.generate_grasp_matrix()

        # TODO: Evaluate grasp quality
        self.evaluate_grasp_quality()

        # TODO: Search for optimal grasp
        self.search_optimal_grasp()

        response.success = True
        response.message = "Grasp pipeline executed."
        return response

    # TODO: Implement grasp matrix generation
    def generate_grasp_matrix(self) -> None:
        pass

    # TODO: Implement grasp quality evaluation
    def evaluate_grasp_quality(self) -> None:
        pass

    # TODO: Implement optimal grasp search
    def search_optimal_grasp(self) -> None:
        pass

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GraspPipelineManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()