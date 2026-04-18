import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.x_sub = self.create_subscription(Float64, '/plant_state', self.state_callback, 10)
        self.y_pub = self.create_publisher(Float64, '/measured_y', 10)

    def state_callback(self, msg):
        # Envío de la señal limpia (sin ruido)
        out = Float64()
        out.data = msg.data
        self.y_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()