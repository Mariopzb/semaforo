#!/usr/bin/env python
import rclpy, time, math
from rclpy import qos
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32  # Para el puzzlebot

class TurtleOdometryArc(Node):
    def __init__(self):
        super().__init__("odometry_arc_node")
        self.get_logger().info("Robot pose estimated by odometry with arc integration")
        # Robot parameters for puzzlebot
        self.r = 0.05
        self.L = 0.19
        self.v = 0.0
        self.w = 0.0
        self.wR = 0.0
        self.wL = 0.0
        self.rate = 100.0
        self.x, self.y, self.theta = 5.5, 5.5, 0.0   # If turtlesim
        # self.x, self.y, self.theta = 0.0, 0.0, 0.0   # If puzzlebot
        self.pub = self.create_publisher(Pose, "/odom", 1)
        self.timer = self.create_timer(1.0 / self.rate, self.callback_odometry)

        # Para el turtlesim, comentar una o la otra dependiendo de cual se quiera usar
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 1)

        # Para el puzzlebot
        self.create_subscription(Float32, "/VelocityEncR", self.callback_wR, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, "/VelocityEncL", self.callback_wL, qos.qos_profile_sensor_data)
        self.t0 = time.time()

    def callback_odometry(self):
        elapsed_time = time.time() - self.t0
        self.t0 = time.time()
        # Calcular self.v y self.w para el puzzlebot
        self.v = self.r * (self.wR + self.wL) / 2
        self.w = self.r * (self.wR - self.wL) / self.L

        # Integración en arcos
        delta_s = self.v * elapsed_time
        delta_theta = self.w * elapsed_time
        self.x += delta_s * math.cos(self.theta + delta_theta / 2)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalizar ángulo

        msg = Pose()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pub.publish(msg)

    def callback_wR(self, msg):
        self.wR = msg.data

    def callback_wL(self, msg):
        self.wL = msg.data

    def pose_callback(self, msg):
        self.v = msg.linear_velocity
        self.w = msg.angular_velocity

def main(args=None):
    rclpy.init(args=args)
    nodeh = TurtleOdometryArc()
    try:
        rclpy.spin(nodeh)
    except KeyboardInterrupt:
        print("Terminado por el usuario!!")
    finally:
        nodeh.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()