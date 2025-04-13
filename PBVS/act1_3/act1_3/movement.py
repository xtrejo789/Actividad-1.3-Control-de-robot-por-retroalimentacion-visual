import sys
import time
import math
import cv2
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

        # PID parameters
        self.declare_parameter('Kp', 0.2)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.1)
        self.declare_parameter('Kp_angular', 0.0)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.Kp_angular = self.get_parameter('Kp_angular').value

        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info(f"PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")
        self.get_logger().info(f"Angular PID parameter: Kp_angular={self.Kp_angular}")

        # CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        # Obtener handles
        self.robot = self.sim.getObject('/PioneerP3DX')
        self.sphere = self.sim.getObject('/Sphere')
        self.cam = self.sim.getObject('/PioneerP3DX/visionSensor')
        self.get_logger().info("📷 Connected to visionSensor")

        # Publisher /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("⏳ Waiting for simulation to start...")
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)
        self.get_logger().info("▶️ Simulation started")

        self.prev_error = 0.0
        self.integral = 0.0

        self.timer = self.create_timer(0.1, self.follow_sphere)

    def follow_sphere(self):
        try:
            pos_robot = self.sim.getObjectPosition(self.robot, -1)
            pos_sphere = self.sim.getObjectPosition(self.sphere, -1)
            orientation = self.sim.getObjectOrientation(self.robot, -1)
            theta = orientation[2]

            dx = pos_sphere[0] - pos_robot[0]
            dy = pos_sphere[1] - pos_robot[1]
            distance_error = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)

            self.integral += distance_error
            derivative = distance_error - self.prev_error
            self.prev_error = distance_error

            linear = self.Kp * distance_error + self.Ki * self.integral + self.Kd * derivative
            angle_error = math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta))
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = self.Kp_angular * angle_error

            self.cmd_pub.publish(cmd)

            self.get_logger().info(f"📍 Robot to sphere → dx={dx:.2f}, dy={dy:.2f}, err={distance_error:.2f}")
            self.get_logger().info(f"🚀 Enviando cmd_vel — linear.x: {cmd.linear.x:.3f}, angular.z: {cmd.angular.z:.3f}")

            self.sim.handleVisionSensor(self.cam)
            img, resolution = self.sim.getVisionSensorImg(self.cam, 0)
            img = np.frombuffer(img, dtype=np.uint8).reshape((resolution[1], resolution[0], 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.flip(img, 0)
            cv2.imshow("VisionSensor", img)
            cv2.waitKey(1)
            self.sim.step()

        except Exception as e:
            self.get_logger().error(f"❌ Error in follow_sphere: {e}")

    def param_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                self.Kp = param.value
            elif param.name == 'Ki':
                self.Ki = param.value
            elif param.name == 'Kd':
                self.Kd = param.value
            elif param.name == 'Kp_angular':
                self.Kp_angular = param.value
        self.get_logger().info(f"Updated PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, Kp_angular={self.Kp_angular}")
        self.get_logger().info("✅ Parameters updated")
        return SetParametersResult(successful=True)

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
