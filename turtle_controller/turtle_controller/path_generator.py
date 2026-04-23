#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from turtlesim.msg import Pose

class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")
        
        
        self.point_list = [
            [2.0, 0.0], 
            [2.0, 2.0], 
            [0.0, 2.0], 
            [0.0, 0.0]
        ] 
        
    #triangulo
    
      #  self.point_list = [
      #  [2.0, 0.0], 
      #  [1.0, 1.732], 
      #  [0.0, 0.0]]
    
    
    #trapecio
        #self.point_list = [
       # [2.0, 0.0], 
        #[1.5, 1.0], 
        #[0.5, 1.0], 
        #[0.0, 0.0]]
   #     self.point_list = [
    #        [0.0, 0.0],
     #       [0.5, 0.95],
      #      [1.0, 0.0],
       #     [0.0, 0.59],
        #    [0.0, 0.0]
       # ]   


       
        self.current_goal_idx = 0
        self.threshold = 0.1  # Tolerancia para considerar que llegó al punto (en metros)
        
        # Publicador del punto objetivo
        self.pub = self.create_publisher(Pose, "/next_point", 1)
        
        # Suscriptor a la odometría para saber dónde está el robot
        self.create_subscription(Pose, "/odom", self.odom_callback, 1)
        
        # Timer para publicar el objetivo actual frecuentemente
        self.create_timer(0.5, self.publish_goal)
        
        self.get_logger().info("Generador de trayectoria para cuadrado de 2m iniciado.")

    def publish_goal(self):
        """Publica el objetivo actual mientras no se haya terminado la lista."""
        if self.current_goal_idx < len(self.point_list):
            msg = Pose()
            msg.x = float(self.point_list[self.current_goal_idx][0])
            msg.y = float(self.point_list[self.current_goal_idx][1])
            self.pub.publish(msg)
        else:
            self.get_logger().info("¡Trayectoria cuadrada completada con éxito!")
            # Opcional: rclpy.shutdown() si solo quieres que lo haga una vez

    def odom_callback(self, msg):
        """Revisa la distancia entre el robot y el punto objetivo actual."""
        if self.current_goal_idx < len(self.point_list):
            target_x = self.point_list[self.current_goal_idx][0]
            target_y = self.point_list[self.current_goal_idx][1]
            
            # Cálculo de la distancia euclidiana
            distance = math.sqrt((target_x - msg.x)**2 + (target_y - msg.y)**2)
            
            if distance < self.threshold:
                self.get_logger().info(f"Punto {self.current_goal_idx + 1} alcanzado ({target_x}, {target_y})")
                self.current_goal_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nGenerador detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()