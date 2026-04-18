import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ManagerNode(Node):
    def __init__(self):
        super().__init__('manager_node')
        self.ref_pub = self.create_publisher(Float64, '/reference', 10)
        # Temporizador de 3 segundos por nivel de referencia
        self.timer = self.create_timer(3.0, self.update_reference)
        self.references = [0.0, 1.0, 2.0, -1.0, 0.5]
        self.index = 0

    def update_reference(self):
        msg = Float64()
        msg.data = self.references[self.index]
        self.ref_pub.publish(msg)
        self.get_logger().info(f'Referencia: {msg.data:.2f} rad')
        self.index = (self.index + 1) % len(self.references)

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()