
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist
import random
from math import pi



class LineFollowerController:

	def __init__(self, name):
		self.thymio_name = name
		self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(10)


	def RotateByTheta(self, theta):
		vel_msg = Twist()
		angSpeed = 0.2
		vel_msg.linear.x = 0.0 # m/s
		if theta < pi/3:
			angSpeed = 0.1
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
		if distance < 0.2:
			vel_msg.linear.x = 0.05 # m/s
		else:
			vel_msg.linear.x = 0.10
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

	def Move(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0.1 # m/s
		self.velocity_publisher.publish(vel_msg)

	def Rotate(self, ccw):
		vel_msg = Twist()
		if ccw == 1:
			vel_msg.angular.z = 0.1 # m/s
		else:
			vel_msg.angular.z = -0.1 # m/s
		self.velocity_publisher.publish(vel_msg)

	def Stop(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0.
		self.velocity_publisher.publish(vel_msg)  


	def RandomWalker(self):
		randMove = random.randrange(0, 2, 1)
		print(randMove)
		if(randMove != 0):
			dist = random.uniform(0.1, 0.5)
			print ("dist ", dist)
			self.MoveDistance(dist)
		else:
			theta = random.uniform(-pi/4, pi/4)
			print ("theta ", theta)
			self.RotateByTheta(theta)

	
