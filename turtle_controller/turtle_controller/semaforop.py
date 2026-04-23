import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficSupervisor(Node):
    def __init__(self):
        super().__init__('traffic_supervisor')
        self.get_logger().info("Supervisor de Semáforo Iniciado")

        self.bridge = CvBridge()

        self.STATE_GREEN  = "GREEN"
        self.STATE_YELLOW = "YELLOW"
        self.STATE_RED    = "RED"
        self.current_state    = self.STATE_GREEN
        self.speed_multiplier = 1.0

        #Historial de colores para saber cuales se han visto desde el último verde
        self.seen_yellow = False
        self.seen_red    = False

        #Subs
        self.sub_cam = self.create_subscription(
            Image, '/video_source/raw', self.image_callback, 10)
        self.sub_vel = self.create_subscription(
            Twist, '/cmd_vel_raw', self.velocity_callback, 10)

        # Publica la velocidad al puzzle
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)


    # Detección de color

    def has_color(self, mask, min_area=600):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > min_area:
                return True
        return False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        m_green  = cv2.inRange(hsv, np.array([40,  50,  50]), np.array([80,  255, 255]))
        m_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([35,  255, 255]))
        m_red    = cv2.inRange(hsv, np.array([  0, 150, 120]), np.array([ 10, 255, 255])) | \
                   cv2.inRange(hsv, np.array([170, 150, 120]), np.array([180, 255, 255]))

        detected_green  = self.has_color(m_green)
        detected_yellow = self.has_color(m_yellow)
        detected_red    = self.has_color(m_red)

        self.update_state(detected_green, detected_yellow, detected_red)


        self.get_logger().info(f"Estado: {self.current_state}", throttle_duration_sec=1.0)

    def update_state(self, green, yellow, red):
        prev_state = self.current_state

        if self.current_state == self.STATE_GREEN:
            if yellow and not green:
                self.current_state = self.STATE_YELLOW
                self.seen_yellow   = True
                self.speed_multiplier = 0.5

        elif self.current_state == self.STATE_YELLOW:
            if red and self.seen_yellow:
                self.current_state = self.STATE_RED
                self.seen_red      = True
                self.speed_multiplier = 0.0

        elif self.current_state == self.STATE_RED:
            if green and self.seen_red:
                self.current_state = self.STATE_GREEN
                self.seen_yellow   = False
                self.seen_red      = False
                self.speed_multiplier = 1.0

        #Publica los cambios de estado
        if self.current_state != prev_state:
            self.get_logger().info(f"Cambio de estado: {prev_state} → {self.current_state}")

    def velocity_callback(self, msg):
        final_vel = Twist()
        final_vel.linear.x  = msg.linear.x  * self.speed_multiplier
        final_vel.angular.z = msg.angular.z * self.speed_multiplier
        self.pub_vel.publish(final_vel)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()