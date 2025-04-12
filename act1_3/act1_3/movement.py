import sys
import time
import cv2
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

# ZMQ API path
sys.path.append('/home/ximena/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SphereChaser(Node):
    def __init__(self):
        super().__init__('movement')

        # PID params as dynamically changeable parameters
        self.declare_parameter('Kp', 0.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # Register a callback to handle updates
        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info(f"PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

        # Conexión a CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        # Obtener sensor de visión
        self.cam = self.sim.getObject('/PioneerP3DX/visionSensor')
        self.get_logger().info("📷 Connected to visionSensor")

        # Publicador al tópico /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Esperar a que comience la simulación
        self.get_logger().info("⏳ Waiting for simulation to start...")
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)
        self.get_logger().info("▶️ Simulation started")

        # Variables PID
        self.prev_error = 0.0
        self.integral = 0.0

        # Timer para seguimiento
        self.timer = self.create_timer(0.1, self.track_sphere)

    def track_sphere(self):
        try:
            self.sim.handleVisionSensor(self.cam)
            img, resolution = self.sim.getVisionSensorImg(self.cam, 0)
            resX, resY = resolution

            img = np.frombuffer(img, dtype=np.uint8).reshape((resY, resX, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 0)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([20, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            M = cv2.moments(mask)

            cmd = Twist()

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                error = (resX // 2) - cx

                # PID control
                self.integral += error
                derivative = error - self.prev_error
                self.prev_error = error

                control = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

                cmd.linear.x = 0.2
                cmd.angular.z = control

                self.get_logger().info(
                    f"🎯 Sphere detected — Error: {error} | P={self.Kp * error:.3f}, I={self.Ki * self.integral:.3f}, D={self.Kd * derivative:.3f}"
                )
            else:
                self.get_logger().info("🧐 Sphere not visible")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.integral = 0.0
                self.prev_error = 0.0
            
            self.get_logger().info(f"🚀 Enviando cmd_vel — linear.x: {cmd.linear.x:.3f}, angular.z: {cmd.angular.z:.3f}")
            self.cmd_pub.publish(cmd)
            cv2.imshow("VisionSensor", img)
            cv2.waitKey(1)
            self.sim.step()

        except Exception as e:
            self.get_logger().error(f"❌ Error in track_sphere: {e}")

    def param_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
            elif param.name == 'Ki':
                self.Ki = param.value
            elif param.name == 'Kd':
                self.Kd = param.value
        self.get_logger().info(f"Updated PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        return SetParametersResult(successful=True)
        # Log the updated parameters

def main(args=None):
    rclpy.init(args=args)
    node = SphereChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
