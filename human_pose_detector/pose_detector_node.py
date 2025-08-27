import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class PoseDetectorNode(Node):
    def __init__(self):
        super().__init__('pose_detector_node')
        self.get_logger().info('Nó de detecção de pose iniciado...')

        # Inicializa o MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils

        # Inicializa o CV_Bridge
        self.bridge = CvBridge()

        # Cria o subscriber para a imagem da câmera
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Tópico da câmera RealSense
            self.image_callback,
            10)
        self.subscription  # Evita aviso de variável não utilizada

    def image_callback(self, msg):
        self.get_logger().info('Recebendo imagem...')
        try:
            # Converte a mensagem ROS Image para uma imagem OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Falha ao converter imagem: {e}')
            return

        # Processa a imagem com o MediaPipe
        # Para melhorar a performance, marcamos a imagem como não-gravável
        cv_image.flags.writeable = False
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        cv_image.flags.writeable = True

        # Desenha o esqueleto na imagem
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                cv_image,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                )

        # Mostra a imagem em uma janela
        cv2.imshow("Detector de Pose MediaPipe", cv_image)
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
