import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import message_filters
import tf2_ros
import pyrealsense2 as rs

class PoseTFPublisherNode(Node):
    def __init__(self):
        super().__init__('pose_tf_publisher_node')
        self.get_logger().info('Nó publicador de TF de Pose 3D iniciado.')

        # --- Inicializações ---
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.bridge = CvBridge()
        # ***** LINHA CORRIGIDA ABAIXO *****
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.intrinsics = None

        # --- Subscribers ---
        self.info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10)

        self.color_subscriber = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_subscriber = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber], 10, 0.1)
        self.time_synchronizer.registerCallback(self.image_callback)
        
        self.get_logger().info('Aguardando CameraInfo e imagens...')

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.none
            self.get_logger().info('Parâmetros intrínsecos da câmera recebidos!')
            self.destroy_subscription(self.info_subscriber)

    def image_callback(self, color_msg, depth_msg):
        if self.intrinsics is None:
            self.get_logger().warn('Aguardando parâmetros intrínsecos da câmera... Imagem ignorada.')
            return

        try:
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Falha ao converter imagens: {e}')
            return

        image_rgb = cv2.cvtColor(cv_color_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            h, w, _ = cv_color_image.shape
            nose_landmark = results.pose_landmarks.landmark[0]

            if nose_landmark.visibility > 0.5:
                pixel_x = int(nose_landmark.x * w)
                pixel_y = int(nose_landmark.y * h)
                
                if 0 <= pixel_x < w and 0 <= pixel_y < h:
                    depth_m = cv_depth_image[pixel_y, pixel_x] / 1000.0

                    if depth_m > 0:
                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_x, pixel_y], depth_m)
                        
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'camera_color_optical_frame'
                        t.child_frame_id = 'human_nose'
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = point_3d[0], point_3d[1], point_3d[2]
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
