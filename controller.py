import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.ref_sub = self.create_subscription(Float64, '/reference', self.ref_callback, 10)
        self.y_sub = self.create_subscription(Float64, '/measured_y', self.y_callback, 10)
        self.u_pub = self.create_publisher(Float64, '/control_u', 10)
        
        self.r = 0.0
        self.y = 0.0
        self.kp = 0.5  # Tu ganancia actual
        self.ki = 0.0
        self.integral_e = 0.0
        
        self.dt = 0.01 # Sincronizado a 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

    def ref_callback(self, msg): self.r = msg.data
    def y_callback(self, msg): self.y = msg.data

    def control_loop(self):
        e = self.r - self.y
        self.integral_e += e * self.dt
        u = (self.kp * e) + (self.ki * self.integral_e)
        
        # Saturación [-24, 24]
        u_sat = max(-24.0, min(24.0, u))
        
        msg = Float64()
        msg.data = u_sat
        self.u_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()