import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')
        self.get_logger().info("Nó Subscriber iniciado. Aguardando coordenadas...")
        self.subscription = self.create_subscription(
            Point,
            'human_pose/nose_3d',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Recebi as coordenadas do nariz: x={msg.x:.2f}m, y={msg.y:.2f}m, z={msg.z:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
