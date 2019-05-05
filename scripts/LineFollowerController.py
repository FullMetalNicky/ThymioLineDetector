
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist
import random
from math import pi




class PID:
	
	def __init__(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.last_e = None
		self.sum_e = 0
		
	def step(self, e, dt):
		""" dt should be the time interval from the last method call """
		if(self.last_e is not None):
			derivative = (e - self.last_e) / dt
		else:
			derivative = 0
		self.last_e = e
		self.sum_e += e * dt
		return self.Kp * e + self.Kd * derivative + self.Ki * self.sum_e

	def reset(self):
		self.last_e = 0
		self.sum_e = 0


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
		print(currDist)

	def Move(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0.1 # m/s
		self.velocity_publisher.publish(vel_msg)

	def Rotate(self):
		vel_msg = Twist()
		vel_msg.angular.z = 0.1 # m/s
		self.velocity_publisher.publish(vel_msg)

	def Stop(self):
		vel_msg = Twist()
		vel_msg.linear.x = 0.
		self.velocity_publisher.publish(vel_msg)  


	def RandomWalker(self):
		randMove = random.randrange(0, 2, 1)
		print(randMove)
		if(randMove == 0):
			dist = random.uniform(0.1, 2.0)
			print ("dist ", dist)
			self.MoveDistance(dist)
		else:
			theta = random.uniform(-pi/2, pi/2)
			print ("theta ", theta)
			self.RotateByTheta(theta)

	def get_distance(self, goal_x, goal_y):
		distance = np.sqrt(np.pow((goal_x - self.pose.x), 2) + np.pow((goal_y - self.pose.y), 2))
		return distance

	def MoveToGoal(self, x, y, tol):
		#goal_pose = Pose()
		#goal_pose.point.x = x
		#goal_pose.y = y
		#goal_pose.theta = 0;
		goal_pose_x = x
		goal_pose_y= y
		goal_pose_theta = theta
		distance_tolerance = tol
		vel_msg = Twist()
		speed_controller = PID(1, 0, 0)
		theta_controller = PID(-4, 0, 0)

		t0 = rospy.Time.now().to_sec()
		while self.get_distance(goal_pose_x, goal_pose_y) >= distance_tolerance:

			if self.AssessThreat():
				break
			dist2Target = self.get_distance(goal_pose_x, goal_pose_y)
			angleDiff = self.pose.theta - atan2(goal_pose_y - self.pose.y, goal_pose_x- self.pose.x)
			angleDiff = atan2(sin(angleDiff), cos(angleDiff))

			t1=rospy.Time.now().to_sec()
			speed = speed_controller.step(dist2Target, t0-t1)
			theta = theta_controller.step(angleDiff, t0-t1)
			
			vel_msg.linear.x = speed
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = theta
			
			#This hack is from Simone's PID. Not all heros wear capes... 
			# from here:
			if(theta  < pi/4) or (theta > -pi/4):
				vel_msg.linear.x = speed
			else:
				vel_msg.linear.x = 0
				speed_controller.reset()
			# Until here

			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()

			t0 = t1

		vel_msg.linear.x = 0
		vel_msg.angular.z =0
		self.velocity_publisher.publish(vel_msg)
