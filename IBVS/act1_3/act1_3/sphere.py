import math
import time
import rclpy
from rclpy.node import Node

# Conexión ZMQ
import sys
sys.path.append('/home/ximena/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SphereMover(Node):
    def __init__(self):
        super().__init__('sphere')

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        self.sphere = self.sim.getObject('/Sphere')
        self.get_logger().info("✅ Conectado a esferaNaranja")

        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)

        self.get_logger().info("▶️ Simulación corriendo")

        self.t = 0.0
        self.amplitud = 1.4   # amplitud en X (oscila izquierda-derecha)
        self.velocidad_y = 0.1  # velocidad de avance constante en Y
        self.altura = 0.15

        self.timer = self.create_timer(0.05, self.update_position)

    def update_position(self):
        x = self.amplitud * math.sin(0.25 * self.t)
        y = self.velocidad_y * self.t
        z = self.altura

        self.sim.setObjectPosition(self.sphere, -1, [x, y, z])
        self.sim.step()

        self.get_logger().info(f"🌐 Posición esfera → x={x:.2f}, y={y:.2f}")
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = SphereMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
