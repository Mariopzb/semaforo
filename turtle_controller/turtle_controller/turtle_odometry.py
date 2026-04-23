#!/usrbin/env python
import rclpy, time, math
from rclpy import qos
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32 #Para el puzzlebot


class TurtleOdometry(Node):
	def __init__(self):
		super().__init__("odometry_node")
		self.get_logger().info("Robot pose estimated by odometry")
		#Robot parameters for puzzlebot
		self.r = 0.052
		self.L = 0.18
		self.v = 0.0
		self.w = 0.0
		self.wR = 0.0
		self.wL = 0.0
		self.rate = 100.0
		#self.x, self.y, self.theta = 5.5, 5.5, 0.0   #If turtlesim
		self.x, self.y, self.theta = 0.0, 0.0, 0.0   #If puzzlebot
		self.pub = self.create_publisher(Pose, "/odom", 1)
		self.timer = self.create_timer(1.0/self.rate, self.callback_odometry)

		#------------------------------Para el turtlesim, comentar una o la otra dependiendo de cual se quiera usar-------------------------
		#self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 1)

		#Para el puzzlebot
		self.create_subscription(Float32, "/VelocityEncR", self.callback_wR, qos.qos_profile_sensor_data) #Utilizaremos QoS para el puzzlebot, siempre se hace cuando queremos hacer una conexion de otra computadora
		self.create_subscription(Float32, "/VelocityEncL", self.callback_wL, qos.qos_profile_sensor_data) #Utilizaremos QoS para el puzzlebot, siempre se hace cuando queremos hacer una conexion de otra computadora
		self.t0 = time.time()

	def callback_odometry(self):
		elapsed_time = time.time() - self.t0
		self.t0 = time.time()
		#Calcular self.v y self.w para el puzzlebot
		self.v = self.r*(self.wR + self.wL)/2
		self.w = self.r*(self.wR - self.wL)/self.L
		self.x += elapsed_time * self.v * math.cos(self.theta)
		self.y += elapsed_time * self.v * math.sin(self.theta)
		self.theta += elapsed_time * self.w
		self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) #Si llegamos al angulo este deja de hacerse mas grande que 360
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
		pass
	


def main(args=None):
	rclpy.init(args=args)
	nodeh = TurtleOdometry()
	
	
	try: rclpy.spin(nodeh)
	except KeyboardInterrupt: print("Terminado por el usuario!!")
	

if __name__== "__main__":
	main()
