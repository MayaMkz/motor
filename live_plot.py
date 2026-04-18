import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from collections import deque

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.ref_sub = self.create_subscription(Float64, '/reference', self.ref_callback, 10)
        self.y_sub = self.create_subscription(Float64, '/measured_y', self.y_callback, 10)
        self.u_sub = self.create_subscription(Float64, '/control_u', self.u_callback, 10)

        self.ref = 0.0
        self.y = 0.0
        self.u = 0.0
        self.t = 0.0
        
        # Se define la variable ANTES de usarla en el timer
        self.plot_dt = 0.05 

        self.time_data = deque(maxlen=600)
        self.ref_data = deque(maxlen=600)
        self.y_data = deque(maxlen=600)
        self.u_data = deque(maxlen=600)

        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 10))
        
        # El timer ahora reconoce self.plot_dt correctamente
        self.timer = self.create_timer(self.plot_dt, self.update_plot)

    def ref_callback(self, msg): self.ref = msg.data
    def y_callback(self, msg): self.y = msg.data
    def u_callback(self, msg): self.u = msg.data

    def update_plot(self):
        # El tiempo avanza con la misma variable del refresco
        self.t += self.plot_dt 
        self.time_data.append(self.t)
        self.ref_data.append(self.ref)
        self.y_data.append(self.y)
        self.u_data.append(self.u)

        self.ax1.clear()
        self.ax2.clear()

        # Plot 1: Posición
        self.ax1.plot(self.time_data, self.ref_data, label='Reference r(t)', color='blue')
        self.ax1.plot(self.time_data, self.y_data, label='Output y(t)', color='orange')
        self.ax1.set_ylabel('Position [rad]')
        self.ax1.set_title('Tracking: Reference vs Output')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)

        # Plot 2: Voltaje
        self.ax2.plot(self.time_data, self.u_data, label='Control Input u(t)', color='green')
        self.ax2.axhline(24, color='red', linestyle='--', alpha=0.5)
        self.ax2.axhline(-24, color='red', linestyle='--', alpha=0.5)
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Voltage [V]')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)

        plt.tight_layout()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()