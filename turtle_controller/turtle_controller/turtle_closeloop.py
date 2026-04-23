#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_close_loop_controller")
        self.get_logger().info("Controlador de Lazo Cerrado Iniciado")
        
        #self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.pub = self.create_publisher(Twist, "/cmd_vel_raw", 1)
        self.create_subscription(Pose, "/odom", self.odom_callback, 1)
        self.create_subscription(Pose, "/next_point", self.target_callback, 1)
        
        self.create_timer(0.02, self.state_machine)

        # Parámetros para el reto de 2m
        self.v_limit = 0.4  
        self.Kw = 1.2
        self.Kv = 0.5
        self.tolerance_distance = 0.02
        self.tolerance_angle = 0.03
        self.state = "stop"
        self.end_of_accion = False
        self.got_target = False
        # Inicializar en 0.0 para el reto
        self.x, self.y, self.theta = 0.0, 0.0, 0.0 

    def odom_callback(self, msg):
        self.x, self.y, self.theta = msg.x, msg.y, msg.theta

    def target_callback(self, msg):
        if not self.got_target:
            self.target_x, self.target_y = msg.x, msg.y
            self.got_target = True
            self.state = "state1" # Empezar a girar
            self.get_logger().info(f"Nuevo objetivo recibido: ({msg.x}, {msg.y})")

    def state_machine(self):
        if self.state == "state1":
            self.go_to_angle()
            if self.end_of_accion:
                self.state = "state2"
                self.end_of_accion = False
        
        elif self.state == "state2":
            self.go_to_point()
            if self.end_of_accion:
                self.state = "stop"
                self.end_of_accion = False
                self.got_target = False # Ahora sí, pedimos el siguiente punto
        
        elif self.state == "stop":
            self.stop_robot()

    def go_to_angle(self):
        msg = Twist()
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        target_theta = math.atan2(dy, dx)
        e_theta = target_theta - self.theta
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))

        if abs(e_theta) > self.tolerance_angle:
            msg.angular.z = max(min(self.Kw * e_theta, 0.5), -0.5)
            self.pub.publish(msg)
        else:
            self.stop_robot()
            time.sleep(0.2)
            self.end_of_accion = True

    def go_to_point(self):
        msg = Twist()
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Corrección de ángulo constante mientras avanza
        target_theta = math.atan2(dy, dx)
        e_theta = target_theta - self.theta
        e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))

        if distance > self.tolerance_distance:
            msg.linear.x = min(self.v_limit, self.Kv * distance)
            msg.angular.z = self.Kw * e_theta 
            self.pub.publish(msg)
        else:
            self.stop_robot()
            self.end_of_accion = True

    def stop_robot(self):
        self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()