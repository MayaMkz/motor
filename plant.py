import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class PlantNode(Node):
    def __init__(self):
        super().__init__('plant_node')
        self.u_sub = self.create_subscription(Float64, '/control_u', self.u_callback, 10)
        self.x_pub = self.create_publisher(Float64, '/plant_state', 10)
        
        self.u = 0.0
        self.theta = 0.0
        self.omega = 0.0

        # Parámetros del motor 148867
        A = 200.0      
        B = 12168.4    
        self.dt = 0.01 # Frecuencia de 100 Hz
        
        # --- Discretización Exacta (Zero-Order Hold) ---
        self.Ad = math.exp(-A * self.dt)
        term = (1.0 - self.Ad) / A
        
        self.phi_theta_omega = term
        self.gamma_theta_u = (B / A) * (self.dt - term)
        self.gamma_omega_u = (B / A) * (1.0 - self.Ad)

        self.timer = self.create_timer(self.dt, self.update_plant)

    def u_callback(self, msg):
        self.u = msg.data

    def update_plant(self):
        # Solución analítica de estado: ¡Adiós 1e303!
        new_theta = self.theta + self.phi_theta_omega * self.omega + self.gamma_theta_u * self.u
        new_omega = self.Ad * self.omega + self.gamma_omega_u * self.u
        
        self.theta = new_theta
        self.omega = new_omega

        msg = Float64()
        msg.data = self.theta
        self.x_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlantNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()