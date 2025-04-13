import sys
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.append('/home/ximena/CoppeliaSim_Edu_V4_9_0_rev6_Ubuntu22_04/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class CoppeliaTeleopBridge(Node):
    def __init__(self):
        super().__init__('coppelia_teleop_bridge')

        # ZMQ Connection
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.sim.setStepping(True)
            self.get_logger().info('✅ Connected to CoppeliaSim via ZMQ Remote API')
        except Exception as e:
            self.get_logger().error(f'❌ Could not connect to CoppeliaSim: {str(e)}')
            raise

        # Robot handles
        self.left_motor = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor = self.sim.getObject('/PioneerP3DX/rightMotor')

        # Parameters
        self.wheel_radius = 0.0975
        self.wheel_base = 0.381
        self.v = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.dt = 0.1

        # Subscriptions and Publications
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_v = self.create_publisher(Float32, 'Velocity', 10)
        self.pub_w = self.create_publisher(Float32, 'AngularVelocity', 10)
        self.pub_x = self.create_publisher(Float32, 'X', 10)
        self.pub_y = self.create_publisher(Float32, 'Y', 10)
        self.pub_psi = self.create_publisher(Float32, 'Psi', 10)

        self.create_timer(self.dt, self.update_odometry)

        self.get_logger().info('🚀 Subscribed to /cmd_vel and publishing odometry')

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

        # Differential drive
        v_left = (self.v - self.w * self.wheel_base / 2.0) / self.wheel_radius
        v_right = (self.v + self.w * self.wheel_base / 2.0) / self.wheel_radius

        # Send to CoppeliaSim
        self.sim.setJointTargetVelocity(self.left_motor, v_left)
        self.sim.setJointTargetVelocity(self.right_motor, v_right)
        self.sim.step()

        self.get_logger().info(f'🔁 cmd_vel → v={self.v:.2f} | w={self.w:.2f} | left={v_left:.2f} | right={v_right:.2f}')

    def update_odometry(self):
        self.x += self.v * self.dt * math.cos(self.psi)
        self.y += self.v * self.dt * math.sin(self.psi)
        self.psi = wrap_to_pi(self.psi + self.w * self.dt)

        # Publish
        self.pub_v.publish(Float32(data=self.v))
        self.pub_w.publish(Float32(data=self.w))
        self.pub_x.publish(Float32(data=self.x))
        self.pub_y.publish(Float32(data=self.y))
        self.pub_psi.publish(Float32(data=self.psi))

        self.get_logger().info(
            f'📍 x={self.x:.2f} | y={self.y:.2f} | ψ={math.degrees(self.psi):.2f}°'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
