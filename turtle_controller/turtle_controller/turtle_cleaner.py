#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.create_timer(0.1, self.state_machine)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        
        # Parámetros de movimiento
        self.v = 3.0  
        self.w = 2.0  
        self.t0 = time.time()
        
        self.state = "ADVANCE"
        self.end_of_accion = False
        

        self.current_distance = 0.5
        self.step_increment = 0.4
        #self.triangle_angle = 2.1
        self.triangle_angle = 1.57
        
        # Borde
        self.border_limit = 9.5 

    def state_machine(self):
        if self.end_of_accion:
            if self.state == "ADVANCE":
                self.state = "ROTATE"
            
            elif self.state == "ROTATE":
                self.state = "ADVANCE"
                
                self.current_distance += self.step_increment
                
                #Rebotar
                if self.current_distance > self.border_limit:
                    self.get_logger().info("Choco con el borde, girando")
                    self.current_distance = 1.0 
            
            self.end_of_accion = False
            self.t0 = time.time()


        if self.state == "ADVANCE":
            self.advance(self.current_distance)
        elif self.state == "ROTATE":
            self.rotate(self.triangle_angle)

    def advance(self, desired_distance):
        msg = Twist()
        elapsed_time = time.time() - self.t0
        travelled_distance = self.v * elapsed_time
        
        if travelled_distance < desired_distance:  
            msg.linear.x = self.v
            self.pub.publish(msg)
        else:
            self.stop_motion()
            self.end_of_accion = True  

    def rotate(self, desired_angle):
        msg = Twist()
        elapsed_time = time.time() - self.t0
        travelled_angle = self.w * elapsed_time
        
        if travelled_angle < desired_angle:  
            msg.angular.z = self.w
            self.pub.publish(msg)
        else:
            self.stop_motion()
            self.end_of_accion = True

    def stop_motion(self):
        msg = Twist()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodeh = TurtleController()
    try:
        rclpy.spin(nodeh)
    except KeyboardInterrupt:
        print("\nPrograma terminado.")
    finally:
        nodeh.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()