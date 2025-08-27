import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import message_filters

# Importações necessárias para o QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PoseDetectorNode(Node):
    def __init__(self):
        super().__init__('pose_detector_node_3d')
        self.get_logger().info('Nó de detecção de pose 3D iniciado com QoS customizado...')

        # --- PERFIL DE QOS CUSTOMIZADO PARA SENSORES ---
        # A câmera RealSense publica com um QoS específico para dados de sensores.
        # Precisamos criar um perfil compatível para nossos subscribers.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # ---------------------------------------------

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.bridge = CvBridge()

        # Usamos o perfil de QoS que criamos nos nossos subscribers
        self.color_subscriber = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/color/image_raw', 
            qos_profile=qos_profile)
        
        self.depth_subscriber = message_filters.Subscriber(
            self, 
            Image, 
            '/camera/aligned_depth_to_color/image_raw', 
            qos_profile=qos_profile)

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber], 10, 0.1)
        
        self.time_synchronizer.registerCallback(self.image_callback)
        
        self.get_logger().info('Aguardando imagens de cor e profundidade sincronizadas...')

    def image_callback(self, color_msg, depth_msg):
        self.get_logger().info('Recebendo imagens sincronizadas!') # <-- MENSAGEM DE SUCESSO!
        try:
            cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Falha ao converter imagens: {e}')
            return

        image_rgb = cv2.cvtColor(cv_color_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                cv_color_image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS)
            
            h, w, _ = cv_color_image.shape
            # Pega a coordenada do nariz (ID 0) como exemplo
            nose_landmark = results.pose_landmarks.landmark[0]
            if nose_landmark.visibility > 0.5: # Só calcula se o ponto estiver visível
                pixel_x = int(nose_landmark.x * w)
                pixel_y = int(nose_landmark.y * h)
                
                # Garante que as coordenadas estão dentro dos limites da imagem
                if 0 <= pixel_x < w and 0 <= pixel_y < h:
                    depth_mm = cv_depth_image[pixel_y, pixel_x]
                    depth_m = depth_mm / 1000.0

                    self.get_logger().info(
                        f'Nariz (ID 0): Profundidade Real={depth_m:.2f} metros'
                    )

        cv2.imshow("Detector de Pose 3D", cv_color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
