#!/usrbin/env python
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist



class TurtleController(Node):
	def __init__(self):
		super().__init__("turtle_controller")
		self.create_timer(0.1, self.callback_controller)
		self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
		self.t0 = time.time()
		self.v = 1.0
		self.w = 1.0
		self.desired_distance = 2.0
		self.desired_angle = 3.1416
		
    
	def callback_controller(self):
		self.rotate(3.1416)

	def advance(self, desired_distance):
		msg = Twist()
		elapsed_time = time.time() - self.t0
		travelled_distance =  self.v * elapsed_time
		if travelled_distance < self.desired_distance:	
			msg.linear.x = self.v
			#msg.angular.z = 0.0
			self.pub.publish(msg)
		else:
			print("Got target")
			self.pub.publish(msg)
			rclpy.shutdown()

	def rotate(self, desired_angle):
		msg = Twist()
		elapsed_time = time.time() - self.t0
		travelled_angle =  self.w * elapsed_time
		if travelled_angle< self.desired_angle:	
			msg.angular.z = self.w
			#msg.angular.z = 0.0
			self.pub.publish(msg)
		else:
			print("Got rotation")
			self.pub.publish(msg)
			rclpy.shutdown()
	
	

def main(args=None):
	rclpy.init(args=args)
	nodeh = TurtleController()
	
	
	try: rclpy.spin(nodeh)
	except KeyboardInterrupt: print("Terminado por el usuario!!")
	

if __name__== "__main__":
	main()
