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

class SpherePBVS(Node):
    def __init__(self):
        super().__init__('movement')

        # PID parameters
        self.declare_parameter('Kp_x', 0.0)
        self.declare_parameter('Ki_x', 0.0)
        self.declare_parameter('Kd_x', 0.0)
        self.declare_parameter('Kp_y', 0.0)
        self.declare_parameter('Ki_y', 0.0)
        self.declare_parameter('Kd_y', 0.0)

        self.Kp_x = self.get_parameter('Kp_x').value
        self.Ki_x = self.get_parameter('Ki_x').value
        self.Kd_x = self.get_parameter('Kd_x').value
        self.Kp_y = self.get_parameter('Kp_y').value
        self.Ki_y = self.get_parameter('Ki_y').value
        self.Kd_y = self.get_parameter('Kd_y').value

        self.add_on_set_parameters_callback(self.param_callback)
        self.get_logger().info("PID for X: Kp_x=%.2f, Ki_x=%.2f, Kd_x=%.2f" % (self.Kp_x, self.Ki_x, self.Kd_x))
        self.get_logger().info("PID for Y: Kp_y=%.2f, Ki_y=%.2f, Kd_y=%.2f" % (self.Kp_y, self.Ki_y, self.Kd_y))

        # Camera parameters
        self.cam_fov_deg = 60
        self.cam_res = (256, 256)
        self.cam_z = 0.243  # Altura fija de la camara

        # PID error terms
        self.prev_error_x = 0.0
        self.integral_x = 0.0
        self.prev_error_y = 0.0
        self.integral_y = 0.0

        # CoppeliaSim connection
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.setStepping(True)

        self.robot = self.sim.getObject('/PioneerP3DX')
        self.sphere = self.sim.getObject('/Sphere')
        self.cam = self.sim.getObject('/PioneerP3DX/visionSensor')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("⏳ Waiting for simulation to start...")
        while self.sim.getSimulationState() == 0:
            time.sleep(0.1)
        self.get_logger().info("▶️ Simulation started")

        self.timer = self.create_timer(0.1, self.pbvs_control)

    def estimate_3d_position(self, cx, cy):
        resX, resY = self.cam_res
        fov_rad = math.radians(self.cam_fov_deg)
        alpha_y = fov_rad
        alpha_x = 2 * math.atan(math.tan(alpha_y / 2) * (resX / resY))

        px = (cx - resX / 2) / (resX / 2)
        py = (cy - resY / 2) / (resY / 2)

        x = self.cam_z * math.tan(alpha_x / 2) * px
        y = self.cam_z * math.tan(alpha_y / 2) * py
        return x, y

    def pbvs_control(self):
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
                cy = int(M["m01"] / M["m00"])
                x, y = self.estimate_3d_position(cx, cy)
                self.get_logger().info(f"📐 Error angular (x): {x:.3f} m")

                # PID control for x (angular)
                self.integral_x += x
                derivative_x = x - self.prev_error_x
                self.prev_error_x = x
                cmd.angular.z = -(self.Kp_x * x + self.Ki_x * self.integral_x + self.Kd_x * derivative_x)

                # PID control for y (linear)
                self.integral_y += y
                derivative_y = y - self.prev_error_y
                self.prev_error_y = y
                linear_output = self.Kp_y * y + self.Ki_y * self.integral_y + self.Kd_y * derivative_y

                # Si el error angular es alto, no avanzamos para no perder la esfera
                if abs(x) > 0.01:
                    cmd.linear.x = 0.15
                    self.get_logger().info("⛔ Error angular alto, no avanzamos mucho")
                else:
                    cmd.linear.x = linear_output


                #Para probar primero giro
                #cmd.linear.x = 0.0

                self.get_logger().info(f"🎯 Sphere @ cx={cx}, cy={cy} → x={x:.3f}, y={y:.3f}")
            else:
                self.get_logger().info("🧐 Sphere not visible")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.integral_x = 0.0
                self.prev_error_x = 0.0
                self.integral_y = 0.0
                self.prev_error_y = 0.0

            self.cmd_pub.publish(cmd)
            cv2.imshow("VisionSensor", img)
            cv2.waitKey(1)
            self.sim.step()

        except Exception as e:
            self.get_logger().error(f"❌ Error in pbvs_control: {e}")

    def param_callback(self, params):
        for param in params:
            if param.name == 'Kp_x':
                self.Kp_x = param.value
            elif param.name == 'Ki_x':
                self.Ki_x = param.value
            elif param.name == 'Kd_x':
                self.Kd_x = param.value
            elif param.name == 'Kp_y':
                self.Kp_y = param.value
            elif param.name == 'Ki_y':
                self.Ki_y = param.value
            elif param.name == 'Kd_y':
                self.Kd_y = param.value
        self.get_logger().info(f"✅ PID updated")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = SpherePBVS()
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
