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

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.u_sub = self.create_subscription(Float64, '/control_u', self.u_callback, 10)
        self.theta_pub = self.create_publisher(Float64, '/plant_state', 10)

        self.u = 0.0
        self.theta = 0.0 # Posición [rad]
        self.omega = 0.0 # Velocidad [rad/s]

        # Parámetros físicos del Maxon 148867 (Slide 4)
        self.R = 0.117 # Terminal resistance [Ohm]
        self.J = 1.34e-5 # Rotor inertia [kg*m^2]
        self.kt = 0.0164 # Torque constant [Nm/A]
        self.ke = 0.0301 # Speed constant (convertida a Vs/rad)

        self.dt = 0.01 # Integración a 100 Hz
        self.timer = self.create_timer(self.dt, self.update_plant)

    def u_callback(self, msg):
        self.u = msg.data

    def update_plant(self):
        # Ecuaciones de estado para motor DC (ignorando inductancia L por ser muy pequeña)
        omega_dot = -(self.kt * self.ke) / (self.R * self.J) * self.omega + (self.kt) / (self.R * self.J) * self.u
        
        self.omega += omega_dot * self.dt
        self.theta += self.omega * self.dt

        msg = Float64()
        msg.data = self.theta
        self.theta_pub.publish(msg)

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
        self.x_sub = self.create_subscription(Float64, '/plant_state', self.state_callback, 10)
        self.y_pub = self.create_publisher(Float64, '/measured_y', 10)
        
        # Desviación estándar definida en el PDF
        self.sigma = 0.05 

    def state_callback(self, msg):
        y_true = msg.data
        noise = random.gauss(0, self.sigma)
        y_m = y_true + noise # Salida con ruido

        out = Float64()
        out.data = y_m
        self.y_pub.publish(out)

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
        self.y_sub = self.create_subscription(Float64, '/measured_y', self.y_callback, 10)
        self.u_pub = self.create_publisher(Float64, '/control_u', 10)

        self.r = 0.0
        self.y = 0.0

        # GANANCIAS PI (Cámbialas aquí para tus análisis)
        self.kp = 15.0
        self.ki = 5.0

        self.integral_e = 0.0
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

    def ref_callback(self, msg):
        self.r = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def control_loop(self):
        e = self.r - self.y
        self.integral_e += e * self.dt
        
        u = self.kp * e + self.ki * self.integral_e

        # Saturación de Voltaje y Anti-windup
        if u > 24.0:
            u = 24.0
            self.integral_e -= e * self.dt # Anti-windup
        elif u < -24.0:
            u = -24.0
            self.integral_e -= e * self.dt # Anti-windup

        msg = Float64()
        msg.data = u
        self.u_pub.publish(msg)

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
        self.y_sub = self.create_subscription(Float64, '/measured_y', self.y_callback, 10)
        self.u_sub = self.create_subscription(Float64, '/control_u', self.u_callback, 10)

        self.ref = 0.0
        self.y = 0.0
        self.u = 0.0
        self.t = 0.0
        self.dt = 0.05

        self.time_data = deque(maxlen=400)
        self.ref_data = deque(maxlen=400)
        self.y_data = deque(maxlen=400)
        self.e_data = deque(maxlen=400)
        self.u_data = deque(maxlen=400)

        plt.ion()
        # Crea dos subplots como pide el PDF
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.tight_layout(pad=4.0)
        self.timer = self.create_timer(self.dt, self.update_plot)

    def ref_callback(self, msg): self.ref = msg.data
    def y_callback(self, msg): self.y = msg.data
    def u_callback(self, msg): self.u = msg.data

    def update_plot(self):
        self.t += self.dt
        e = self.ref - self.y

        self.time_data.append(self.t)
        self.ref_data.append(self.ref)
        self.y_data.append(self.y)
        self.e_data.append(e)
        self.u_data.append(self.u)

        self.ax1.clear()
        self.ax1.plot(self.time_data, self.ref_data, 'b--', label='Reference r(t)')
        self.ax1.plot(self.time_data, self.y_data, 'g-', label='Output ym(t)')
        self.ax1.plot(self.time_data, self.e_data, 'r-', alpha=0.5, label='Error e(t)')
        self.ax1.set_title('Plot 1: Tracking & Error Dynamics')
        self.ax1.set_ylabel('Position / Error (rad)')
        self.ax1.legend()
        self.ax1.grid(True)

        self.ax2.clear()
        self.ax2.plot(self.time_data, self.u_data, 'm-', label='Control Input u(t)')
        self.ax2.axhline(24, color='r', linestyle=':', label='Saturation +24V')
        self.ax2.axhline(-24, color='r', linestyle=':', label='Saturation -24V')
        self.ax2.set_title('Plot 2: Control Effort')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Voltage (V)')
        self.ax2.legend()
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
