import cv2
import numpy as np
import rospy

from PatternDetector import PatternDetector
from LineDetector import LineDetector
from LineFollowerController import LineFollowerController
from MazePatterns import MazePatterns
from math import pi
import random


class MazeWalker:
	def __init__(self, name):
		self.thymio_name = name
		self.line_detector = LineDetector(self.thymio_name )
		self.controller = LineFollowerController(self.thymio_name )     
		self.pattern_detector = PatternDetector()
		self.line_detector.Init()
		[w, h] = self.line_detector.GetImageSize()
		self.pattern_detector.SetImageSize(w,h)
		self.center = self.line_detector.GetCenter3DCoords()
		self.state = MazePatterns.noline

	def Align(self):
			frame = self.line_detector.GetTopViewFrame()
			eps = 0.01
			point, direction = self.line_detector.GetLineInRobotFrame(frame)
			while direction is None or ((np.fabs(direction[1]) > eps) and (1.0 - np.fabs(direction[0]) > eps)):
				ccw = 1
				#if direction is not None and direction[1]>0:
				#	ccw = 0
				#	print(direction)
				self.controller.Rotate(ccw)
				frame = self.line_detector.GetCroppedTopViewFrame()
				point, direction = self.line_detector.GetLineInRobotFrame(frame)
			
			self.controller.Stop()

			frame = self.line_detector.GetTopViewFrame()
			offset = self.line_detector.GetLineOffset(frame) 
			if np.fabs(offset) < 0.12:
				return
			elif(offset < 0):
				self.controller.RotateByTheta(-pi/2)
				self.controller.MoveDistance(np.fabs(offset))
				self.controller.RotateByTheta(pi/2)
			elif offset > 0:
				self.controller.RotateByTheta(pi/2)
				self.controller.MoveDistance(np.fabs(offset)/2)
				self.controller.RotateByTheta(-pi/2)


	def RandomWalker(self):
		while not rospy.is_shutdown():
			frame = self.line_detector.GetTopViewFrame()
			patternMat = self.pattern_detector.CreatePatternMatrix(frame)
			state = self.pattern_detector.GetPattern(patternMat)
			if self.state != state:
				print(state)
				self.state = state
			
			if state != MazePatterns.noline:
				self.controller.Stop()
				point, direction = self.line_detector.GetLineInRobotFrame(frame)
				theta = -(np.arctan2(point[0], point[1]) - pi/2)
				distance = np.sqrt(point[0]*point[0] + point[1]*point[1])
				self.controller.RotateByTheta(theta)
				self.controller.MoveDistance(distance)
				self.controller.Stop()
				self.Align()
				rospy.signal_shutdown('Quit')
			else:
				self.controller.RandomWalker()


	def GameLoop(self):
		while self.state != "destination" and not rospy.is_shutdown():
			frame = self.line_detector.GetTopViewFrame()
			patternMat = self.pattern_detector.CreatePatternMatrix(frame)
			state = self.pattern_detector.GetPattern(patternMat)
			if self.state != state:
				print(state)
				print(patternMat)
				

			if state == MazePatterns.noline:
				self.controller.RandomWalker()

			elif state == MazePatterns.paraline:
				if self.state != MazePatterns.paraline:
					print("Align")
					self.Align()
				self.controller.Move()

			elif state == MazePatterns.ortholine:
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				randMove = random.randrange(0, 2, 1)
				if randMove == 1:
					self.controller.RotateByTheta(pi/2)
				else:
					self.controller.RotateByTheta(-pi/2)

			elif state == MazePatterns.crossroads:
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				randMove = random.randrange(0, 3, 1)
				if (randMove == 0):
					self.controller.Move()
				elif randMove == 1:
					self.controller.RotateByTheta(pi/2)
				else:
					self.controller.RotateByTheta(-pi/2)

			elif state == MazePatterns.deadend:
				self.controller.Stop()
				self.controller.RotateByTheta(pi)

			elif state == MazePatterns.destination:
				self.controller.Stop()
				print("great success!")
				rospy.signal_shutdown('Quit')

			elif state == MazePatterns.rightturn:
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				self.controller.RotateByTheta(-pi/2)

			elif state == MazePatterns.leftturn:
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				self.controller.RotateByTheta(pi/2)

			elif state == MazePatterns.weird:
				self.controller.Stop()
				point, direction = self.line_detector.GetLineInRobotFrame(frame)
				theta = -(np.arctan2(point[0], point[1]) - pi/2)
				distance = np.sqrt(point[0]*point[0] + point[1]*point[1])
				self.controller.RotateByTheta(theta)
				self.controller.MoveDistance(distance)
				self.controller.Stop()
				self.Align()

			self.state = state

	