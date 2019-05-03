import cv2
import numpy as np
import rospy

from PatternDetector import PatternDetector
from LineDetector import LineDetector
from LineFollowerController import LineFollowerController

class MazeWalker:
	def __init__(self, name):
		self.thymio_name = name
		self.line_detector = LineDetector(self.thymio_name )
		self.controller = LineFollowerController(self.thymio_name )     
		self.pattern_detector = PatternDetector(5,3)
		self.line_detector.Init()
		[w, h] = self.line_detector.GetImageSize()
		self.pattern_detector.SetImageSize(w,h)
		self.state = "init"

	def Simple(self):
		self.controller.Slow()
		frame = self.line_detector.GetTopViewFrame()
		if frame is not None:
			patternMat = self.pattern_detector.CreatePatternMatrix(frame)
			state = self.pattern_detector.GetPattern(patternMat)
			print(state)


	def GameLoop(self):    	
		while self.state != "destination":
			frame = self.line_detector.GetTopViewFrame()
			patternMat = self.pattern_detector.CreatePatternMatrix(frame)
			state = self.pattern_detector.GetPattern(patternMat)

			if state == "noline":
				self.controller.RandomWander()
			elif state == "paraline":
				self.controller.Move()
			elif state == "ortholine":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(distance)
				#Random choice +/-90
				self.controller.RotateByTheta(theta)
			elif state == "deadend":
				self.controller.RotateByTheta(180)
			elif state == "rightcorner":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(distance)
				self.controller.RotateByTheta(90)
			elif state == "leftcorner":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(distance)
				self.controller.RotateByTheta(-90)
			elif state == "tjunction":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(distance)
				#Random choice +/-90 
				self.controller.RotateByTheta(theta)
			elif state == "crossroads":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(distance)
				#Random choice +/-90 or 0
				if (theta == 0):
					self.controller.Move()
				else:
					self.controller.RotateByTheta(theta)
			elif state == "weird":
				line = self.line_detector.GetLineInRobotFrame(frame)
				#compute theta and distance 
				self.controller.RotateByTheta(theta)
				self.controller.MoveDistance(distance)
				#Random choice +/-90 
				self.controller.RotateByTheta(theta)

			self.Align(frame)


		def Align(self, frame):
			straight = self.line_detector.IsLineStraight(frame)
			if(straight == False):
				line = self.line_detector.GetLineInRobotFrame(frame)
				#compute theta 
				self.controller.RotateByTheta(theta)
			offset = self.line_detector.GetLineOffset(frame)
			if offset < eps:
				return
			elif(offset < 0):
				self.controller.RotateByTheta(90)
				self.controller.MoveDistance(offset)
				self.controller.RotateByTheta(-90)
			elif offset > 0:
				self.controller.RotateByTheta(-90)
				self.controller.MoveDistance(offset)
				self.controller.RotateByTheta(90)