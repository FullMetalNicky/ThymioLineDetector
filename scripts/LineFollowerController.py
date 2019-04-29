
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist



class LineFollowerController:

	def __init__(self, name):
		#self.camera = thymioCamera
		self.thymio_name = name
		self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel', Twist, queue_size=10)
		self.rotate = 1
		self.translate = 1

	def RotateByTheta(self, theta):
		vel_msg = Twist()
		angSpeed = 0.2
		vel_msg.linear.x = 0.0 # m/s
		vel_msg.angular.z = angSpeed # m/s
		currTheta = 0
		t0 = rospy.Time.now().to_sec()

		if(theta < 0):
		    vel_msg.angular.z = -angSpeed # m/s
		    theta = -theta

		while currTheta < theta:
		    t1 = rospy.Time.now().to_sec()
		    currTheta = (t1 - t0)*angSpeed
		    # Publishing thymo vel_msg
		    self.velocity_publisher.publish(vel_msg)
		    # .. at the desired rate.
		    self.rate.sleep()

		# Stop thymio. With is_shutdown condition we do not reach this point.
		vel_msg.linear.x = 0.
		vel_msg.angular.z = 0.
		self.velocity_publisher.publish(vel_msg)

	def MoveDistance(self, distance):
		vel_msg = Twist()
		vel_msg.linear.x = 0.2 # m/s
		currDist =0
		t0 = rospy.Time.now().to_sec()

		while currDist < distance:
			t1 = rospy.Time.now().to_sec()
			currDist = (t1 - t0)*vel_msg.linear.x
			# Publishing thymo vel_msg
			self.velocity_publisher.publish(vel_msg)
			# .. at the desired rate.
			self.rate.sleep()

		# Stop thymio. With is_shutdown condition we do not reach this point.
		vel_msg.linear.x = 0.
		vel_msg.angular.z = 0.
		self.velocity_publisher.publish(vel_msg)  

	def Slow(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0.1 # m/s
		self.velocity_publisher.publish(vel_msg)
