# motor
¡Claro que sí! Vamos a armar este proyecto desde cero en su propia carpeta para que tengas todo súper limpio y listo para generar esos 6 plots que te piden. 

Basándome en los requerimientos del PDF (motor Maxon 148867, controlador PI, saturación a $\pm24$ V, y ruido blanco con $\sigma=0.05$), aquí tienes la guía paso a paso, los comandos y todos los códigos adaptados a la física real de ese motor.

---

### Paso 1: Crear el Workspace y el Paquete

Abre tu terminal y ejecuta los siguientes comandos para crear una carpeta exclusiva para este proyecto y generar el paquete de ROS 2:

```bash
# 1. Crear el workspace
mkdir -p ~/maxon_ws/src
cd ~/maxon_ws/src

# 2. Crear el paquete de ROS 2
ros2 pkg create --build-type ament_python maxon_control --dependencies rclpy std_msgs

# 3. Entrar a la carpeta donde van los scripts
cd maxon_control/maxon_control

# 4. Crear los archivos vacíos de los nodos
touch manager_node.py motor_node.py sensor_node.py controller_node.py plotter_node.py
```

---

### Paso 2: Código de los Nodos

Copia y pega el siguiente código en cada uno de los archivos correspondientes.

#### 1. `manager_node.py` (Genera la referencia cada 3 segundos)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ManagerNode(Node):
    def __init__(self):
        super().__init__('manager_node')
        self.ref_pub = self.create_publisher(Float64, '/reference', 10)
        
        # Cambia la referencia cada 3 segundos
        self.timer = self.create_timer(3.0, self.update_reference)
        
        # Publica la referencia actual a alta velocidad
        self.pub_timer = self.create_timer(0.05, self.publish_ref)

        self.references = [0.0, 1.0, 2.0, -1.0, 0.5]
        self.index = 0
        self.current_ref = self.references[0]

    def update_reference(self):
        self.index = (self.index + 1) % len(self.references)
        self.current_ref = self.references[self.index]
        self.get_logger().info(f'Cambio de referencia: {self.current_ref:.2f} rad')

    def publish_ref(self):
        msg = Float64()
        msg.data = self.current_ref
        self.ref_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. `motor_node.py` (Modelo de 2do orden del Motor Maxon 148867)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.u1_sub = self.create_subscription(Float64, '/control_u1', self.u1_callback, 10)
        self.u2_sub = self.create_subscription(Float64, '/control_u2', self.u2_callback, 10)
        
        self.theta1_pub = self.create_publisher(Float64, '/plant_state1', 10)
        self.theta2_pub = self.create_publisher(Float64, '/plant_state2', 10)

        self.u1 = 0.0; self.theta1 = 0.0; self.omega1 = 0.0
        self.u2 = 0.0; self.theta2 = 0.0; self.omega2 = 0.0

        self.R = 0.316; self.J = 1.34e-5; self.kt = 0.0302; self.ke = 0.0302
        self.dt = 0.01

        self.a = (self.kt * self.ke) / (self.R * self.J)
        self.b = self.kt / (self.R * self.J)
        exp_adt = math.exp(-self.a * self.dt)
        
        self.Ad_11 = 1.0; self.Ad_12 = (1.0 - exp_adt) / self.a
        self.Ad_21 = 0.0; self.Ad_22 = exp_adt
        self.Bd_1 = (self.b / self.a) * (self.dt - self.Ad_12)
        self.Bd_2 = (self.b / self.a) * (1.0 - exp_adt)

        self.timer = self.create_timer(self.dt, self.update_plant)

    def u1_callback(self, msg): self.u1 = msg.data
    def u2_callback(self, msg): self.u2 = msg.data

    def update_plant(self):
        # Sistema 1
        theta1_next = self.Ad_11 * self.theta1 + self.Ad_12 * self.omega1 + self.Bd_1 * self.u1
        omega1_next = self.Ad_21 * self.theta1 + self.Ad_22 * self.omega1 + self.Bd_2 * self.u1
        self.theta1 = theta1_next; self.omega1 = omega1_next
        
        # Sistema 2
        theta2_next = self.Ad_11 * self.theta2 + self.Ad_12 * self.omega2 + self.Bd_1 * self.u2
        omega2_next = self.Ad_21 * self.theta2 + self.Ad_22 * self.omega2 + self.Bd_2 * self.u2
        self.theta2 = theta2_next; self.omega2 = omega2_next

        msg1 = Float64(); msg1.data = self.theta1
        msg2 = Float64(); msg2.data = self.theta2
        self.theta1_pub.publish(msg1)
        self.theta2_pub.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. `sensor_node.py` (Añade el ruido blanco gaussiano)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.x1_sub = self.create_subscription(Float64, '/plant_state1', self.state1_callback, 10)
        self.x2_sub = self.create_subscription(Float64, '/plant_state2', self.state2_callback, 10)
        
        self.y1_pub = self.create_publisher(Float64, '/measured_y1', 10)
        self.y2_pub = self.create_publisher(Float64, '/measured_y2', 10)
        
        # AJUSTA ESTO PARA EL CASO 2 (Ruido vs Sin Ruido): 
        # Sistema 1 con ruido, Sistema 2 limpio (sigma2 = 0.0)
        self.sigma1 = 0.05 
        self.sigma2 = 0.05 

    def state1_callback(self, msg):
        y_m = msg.data + random.gauss(0, self.sigma1)
        out = Float64(); out.data = y_m
        self.y1_pub.publish(out)

    def state2_callback(self, msg):
        y_m = msg.data + random.gauss(0, self.sigma2)
        out = Float64(); out.data = y_m
        self.y2_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4. `controller_node.py` (PI con Saturación $\pm 24V$)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.ref_sub = self.create_subscription(Float64, '/reference', self.ref_callback, 10)
        self.y1_sub = self.create_subscription(Float64, '/measured_y1', self.y1_callback, 10)
        self.y2_sub = self.create_subscription(Float64, '/measured_y2', self.y2_callback, 10)
        
        self.u1_pub = self.create_publisher(Float64, '/control_u1', 10)
        self.u2_pub = self.create_publisher(Float64, '/control_u2', 10)

        self.r = 0.0
        self.y1 = 0.0; self.y2 = 0.0
        self.integral_e1 = 0.0; self.integral_e2 = 0.0
        self.dt = 0.05

        # ==========================================
        # CONFIGURACIÓN DEL SISTEMA 1 (Color Verde)
        # ==========================================
        self.kp1 = 0.17
        self.ki1 = 0.30
        self.sat1_enabled = True

        # ==========================================
        # CONFIGURACIÓN DEL SISTEMA 2 (Color Morado)
        # ==========================================
        self.kp2 = 0.13
        self.ki2 = 0.55
        self.sat2_enabled = True

        self.timer = self.create_timer(self.dt, self.control_loop)

    def ref_callback(self, msg): self.r = msg.data
    def y1_callback(self, msg): self.y1 = msg.data
    def y2_callback(self, msg): self.y2 = msg.data

    def calculate_pi(self, e, integral_e, kp, ki, sat_enabled):
        integral_e += e * self.dt
        u = kp * e + ki * integral_e
        
        if sat_enabled:
            if u > 24.0:
                u = 24.0
                integral_e -= e * self.dt # Anti-windup
            elif u < -24.0:
                u = -24.0
                integral_e -= e * self.dt # Anti-windup
        return u, integral_e

    def control_loop(self):
        e1 = self.r - self.y1
        e2 = self.r - self.y2

        u1, self.integral_e1 = self.calculate_pi(e1, self.integral_e1, self.kp1, self.ki1, self.sat1_enabled)
        u2, self.integral_e2 = self.calculate_pi(e2, self.integral_e2, self.kp2, self.ki2, self.sat2_enabled)

        msg1 = Float64(); msg1.data = u1
        msg2 = Float64(); msg2.data = u2
        self.u1_pub.publish(msg1)
        self.u2_pub.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. `plotter_node.py` (Formato estricto para el Slide 9)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from collections import deque

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.ref_sub = self.create_subscription(Float64, '/reference', self.ref_callback, 10)
        self.y1_sub = self.create_subscription(Float64, '/measured_y1', self.y1_callback, 10)
        self.y2_sub = self.create_subscription(Float64, '/measured_y2', self.y2_callback, 10)
        self.u1_sub = self.create_subscription(Float64, '/control_u1', self.u1_callback, 10)
        self.u2_sub = self.create_subscription(Float64, '/control_u2', self.u2_callback, 10)

        self.ref = 0.0
        self.y1 = 0.0; self.y2 = 0.0
        self.u1 = 0.0; self.u2 = 0.0
        self.t = 0.0; self.dt = 0.05

        self.time_data = deque(maxlen=400); self.ref_data = deque(maxlen=400)
        self.y1_data = deque(maxlen=400); self.y2_data = deque(maxlen=400)
        self.e1_data = deque(maxlen=400); self.e2_data = deque(maxlen=400)
        self.u1_data = deque(maxlen=400); self.u2_data = deque(maxlen=400)

        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.tight_layout(pad=4.0)
        self.timer = self.create_timer(self.dt, self.update_plot)

    def ref_callback(self, msg): self.ref = msg.data
    def y1_callback(self, msg): self.y1 = msg.data
    def y2_callback(self, msg): self.y2 = msg.data
    def u1_callback(self, msg): self.u1 = msg.data
    def u2_callback(self, msg): self.u2 = msg.data

    def update_plot(self):
        self.t += self.dt
        e1 = self.ref - self.y1
        e2 = self.ref - self.y2

        self.time_data.append(self.t); self.ref_data.append(self.ref)
        self.y1_data.append(self.y1); self.y2_data.append(self.y2)
        self.e1_data.append(e1); self.e2_data.append(e2)
        self.u1_data.append(self.u1); self.u2_data.append(self.u2)

        self.ax1.clear()
        self.ax1.plot(self.time_data, self.ref_data, 'b--', label='Reference r(t)')
        self.ax1.plot(self.time_data, self.y1_data, 'g-', label='Output y1 [Sys 1]')
        self.ax1.plot(self.time_data, self.y2_data, 'm-', alpha=0.8, label='Output y2 [Sys 2]')
        self.ax1.plot(self.time_data, self.e1_data, 'r:', alpha=0.5, label='Error e1')
        self.ax1.plot(self.time_data, self.e2_data, 'c:', alpha=0.5, label='Error e2')
        self.ax1.set_title('Plot 1: Tracking & Error Dynamics Comparison')
        self.ax1.set_ylabel('Position / Error (rad)')
        self.ax1.legend(loc='upper right', fontsize='small')
        self.ax1.grid(True)

        self.ax2.clear()
        self.ax2.plot(self.time_data, self.u1_data, 'g-', label='Control u1 [Sys 1]')
        self.ax2.plot(self.time_data, self.u2_data, 'm-', alpha=0.8, label='Control u2 [Sys 2]')
        self.ax2.axhline(24, color='r', linestyle=':', label='Sat +24V')
        self.ax2.axhline(-24, color='r', linestyle=':', label='Sat -24V')
        self.ax2.set_title('Plot 2: Control Effort Comparison')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Voltage (V)')
        self.ax2.legend(loc='upper right', fontsize='small')
        self.ax2.grid(True)

        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Paso 3: Configurar el `setup.py`

Regresa al directorio del paquete (`cd ~/maxon_ws/src/maxon_control`) y abre el archivo `setup.py`. Modifica la sección `entry_points` para que se vea exactamente así:

```python
    entry_points={
        'console_scripts': [
            'manager = maxon_control.manager_node:main',
            'motor = maxon_control.motor_node:main',
            'sensor = maxon_control.sensor_node:main',
            'controller = maxon_control.controller_node:main',
            'plotter = maxon_control.plotter_node:main',
        ],
    },
```

---

### Paso 4: Compilar y Ejecutar

Abre tu terminal en la raíz del workspace (`cd ~/maxon_ws/`) y compila todo:

```bash
colcon build
source install/setup.bash
```

Para correr tu proyecto, necesitas abrir **5 pestañas** de terminal (asegúrate de correr `source install/setup.bash` en cada una) y lanzar los nodos:

1. `ros2 run maxon_control manager`
2. `ros2 run maxon_control motor`
3. `ros2 run maxon_control sensor`
4. `ros2 run maxon_control controller`
5. `ros2 run maxon_control plotter`

---

### ¿Cómo hacer tu PDF para el Slide 8 y 9?
Ya tienes las herramientas. Para generar los **6 plots** que pide el profesor, solo tienes que cambiar el código y volver a correr (no olvides compilar con `colcon build` si cambias algo):

1. **Gain Comparison (2 Plots):** Ejecuta todo con una $K_p=5, K_i=1$ y tómale captura. Luego cambia el código de `controller_node.py` a $K_p=25, K_i=5$, compila, ejecuta y toma captura.
2. **Noise Analysis (2 Plots):** Quédate con unas ganancias fijas. Toma captura normal. Luego ve a `sensor_node.py`, cambia `self.sigma = 0.0` (sin ruido), compila, ejecuta y toma la segunda captura.
3. **Saturation Analysis (2 Plots):** En `controller_node.py`, comenta las líneas del `if u > 24.0:` (para quitar la saturación y dejar que el voltaje suba al infinito). Toma captura y compárala con tu captura del sistema saturado a $\pm24$.
