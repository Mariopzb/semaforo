import rclpy, time, math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class OpenLoopController(Node):
    def __init__(self):
        super().__init__("open_loop_controller")
        self.create_timer(0.1, self.state_machine)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        #self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Pose, "/next_point", self.desired_point_callback, 1)
        
        self.t0 = time.time()
        self.v = 0.8  
        self.w = 2.0 
        self.state = "stop"
        self.end_of_accion = True
        

        self.point_Iam = Pose()
        self.point_Iam.x = 5.5
        self.point_Iam.y = 5.5
        self.point_Iam.theta = 0.0
        
        self.desired_point = Pose()
        self.new_point = False

    def desired_point_callback(self, msg):
        #No procesar el mismo punto
        dist = math.sqrt((msg.x - self.point_Iam.x)**2 + (msg.y - self.point_Iam.y)**2)
        if dist > 0.1 and self.state == "stop":
            self.desired_point = msg
            self.new_point = True
            self.get_logger().info(f"Nuevo punto recibido: x={msg.x}, y={msg.y}")

    def state_machine(self):
        if self.state == "stop" and self.end_of_accion and self.new_point:
            Dx = self.desired_point.x - self.point_Iam.x
            Dy = self.desired_point.y - self.point_Iam.y
            self.desired_distance = math.sqrt(Dx**2 + Dy**2)
            
         
            angle_to_target = math.atan2(Dy, Dx)
            Dq = angle_to_target - self.point_Iam.theta
            self.desired_angle = math.atan2(math.sin(Dq), math.cos(Dq))
            

            self.point_Iam.x = self.desired_point.x
            self.point_Iam.y = self.desired_point.y
            self.point_Iam.theta = angle_to_target
            
            self.new_point = False
            self.state = "rotate"
            self.t0 = time.time()
            self.end_of_accion = False

        elif self.state == "rotate" and self.end_of_accion:
            self.state = "advance"
            self.end_of_accion = False
            self.t0 = time.time()

        elif self.state == "advance" and self.end_of_accion:
            self.state = "stop"
            self.end_of_accion = True 

        # Acciones
        if self.state == "stop": self.stop()
        if self.state == "advance": self.advance(self.desired_distance)
        if self.state == "rotate": self.rotate(self.desired_angle)

    def advance(self, dist):
        msg = Twist()
        elapsed = time.time() - self.t0
        if (self.v * elapsed) < dist:
            msg.linear.x = self.v
            self.pub.publish(msg)
        else:
            self.stop()
            self.end_of_accion = True

    def rotate(self, angle):
        msg = Twist()
        elapsed = time.time() - self.t0

        if (self.w * elapsed) < abs(angle):

            msg.angular.z = self.w if angle > 0 else -self.w
            self.pub.publish(msg)
        else:
            self.stop()
            self.end_of_accion = True

    def stop(self):
        self.pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
