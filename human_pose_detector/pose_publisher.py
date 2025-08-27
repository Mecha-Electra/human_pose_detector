import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped # Importa a mensagem de transformação
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np
import tf2_ros # Importa a biblioteca TF2 do ROS

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.get_logger().info('Nó Publicador de Pose 3D com TF iniciado.')

        # Cria o publisher para as coordenadas 3D do nariz (ainda útil para dados brutos)
        self.publisher_ = self.create_publisher(Point, 'human_pose/nose_3d', 10)

        # --- NOVO: Inicializa o broadcaster de TF ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # --------------------------------------------

        # --- Configuração do MediaPipe ---
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # --- Configuração da RealSense ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        print("[INFO] Iniciando a câmera RealSense...")
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        self.intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        print("[INFO] Câmera iniciada.")

        self.timer = self.create_timer(1.0/30.0, self.timer_callback) # 30 FPS

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(color_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            
            h, w, _ = color_image.shape
            nose_landmark = results.pose_landmarks.landmark[0]

            if nose_landmark.visibility > 0.5:
                pixel_x = int(nose_landmark.x * w)
                pixel_y = int(nose_landmark.y * h)
                
                if 0 <= pixel_x < w and 0 <= pixel_y < h:
                    depth_m = depth_frame.get_distance(pixel_x, pixel_y)
                    
                    if depth_m > 0: # Garante que temos uma leitura de profundidade válida
                        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_x, pixel_y], depth_m)
                        
                        # Publica a mensagem Point como antes
                        msg = Point()
                        msg.x, msg.y, msg.z = point_3d[0], point_3d[1], point_3d[2]
                        self.publisher_.publish(msg)
                        
                        # --- NOVO: Cria e publica a mensagem de Transformação (TF) ---
                        t = TransformStamped()
                        
                        # Cabeçalho da mensagem
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'camera_link' # O frame pai (a base da câmera)
                        t.child_frame_id = 'human_nose'   # O frame filho (o ponto que estamos rastreando)
                        
                        # Translação (posição)
                        t.transform.translation.x = msg.x
                        t.transform.translation.y = msg.y
                        t.transform.translation.z = msg.z
                        
                        # Rotação (como não temos rotação, usamos uma rotação nula)
                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0
                        
                        # Envia a transformação
                        self.tf_broadcaster.sendTransform(t)
                        # -----------------------------------------------------------

                        cv2.putText(color_image, f"Profundidade: {msg.z:.2f}m", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Pose Publisher Node', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Fechando o nó...')
            self.destroy_node()
            rclpy.shutdown()

    def on_shutdown(self):
        print("[INFO] Parando a câmera.")
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
