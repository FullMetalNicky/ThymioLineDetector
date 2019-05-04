import cv2
import numpy as np
import rospy

from PatternDetector import PatternDetector
from LineDetector import LineDetector
from LineFollowerController import LineFollowerController

PI=3.1415926

class MazeWalker:
	def __init__(self, name):
		self.thymio_name = name
		self.line_detector = LineDetector(self.thymio_name )
		self.controller = LineFollowerController(self.thymio_name )     
		self.pattern_detector = PatternDetector(5,3)
		self.line_detector.Init()
		[w, h] = self.line_detector.GetImageSize()
		self.pattern_detector.SetImageSize(w,h)
		self.center = self.line_detector.GetCenter3DCoords()
		self.state = "init"

	def Align(self, frame):
			#straight = self.line_detector.IsLineStraight(frame)
			#print("align")
			eps = 0.1
			point, direction = self.line_detector.GetLineInRobotFrame(frame)
			if (np.fabs(direction[1]) > eps) and (1.0 - np.fabs(direction[0]) > eps):
				theta = np.tan(direction[0]/direction[1])
				self.controller.RotateByTheta(theta)
				print(direction)

			offset = self.line_detector.GetLineOffset(frame) 
			if offset < 0.15:
				return
			elif(offset < 0):
				print(offset)
				self.controller.RotateByTheta(-PI/2)
				self.controller.MoveDistance(offset/2)
				self.controller.RotateByTheta(PI/2)
				print("here")
			elif offset > 0:
				print(offset)
				self.controller.RotateByTheta(PI/2)
				self.controller.MoveDistance(offset/2)
				self.controller.RotateByTheta(-PI/2)
				print("here")

	def Simple(self):
		#self.controller.Stop()
		self.controller.RandomWalker()
		#frame = self.line_detector.GetTopViewFrame()
		#if frame is not None:
			#patternMat = self.pattern_detector.CreatePatternMatrix(frame)
			#state = self.pattern_detector.GetPattern(patternMat)
			#print(state)
			#self.Align(frame)
			#self.controller.RandomWalker()
			#self.controller.Move()


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
				self.controller.MoveDistance(self.center[0])
				#Random choice +/-90
				self.controller.RotateByTheta(theta)
			elif state == "deadend":
				self.controller.RotateByTheta(180)
			elif state == "rightcorner":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				self.controller.RotateByTheta(90)
			elif state == "leftcorner":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				self.controller.RotateByTheta(-90)
			elif state == "tjunction":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				#Random choice +/-90 
				self.controller.RotateByTheta(theta)
			elif state == "crossroads":
				#get center screen coordinates in robot frame
				self.controller.MoveDistance(self.center[0])
				#Random choice +/-90 or 0
				if (theta == 0):
					self.controller.Move()
				else:
					self.controller.RotateByTheta(theta)
			elif state == "weird":
				point, direction = self.line_detector.GetLineInRobotFrame(frame)
				#compute theta and distance 
				theta = np.tan(point[0]/point[1])
				self.controller.RotateByTheta(theta)
				dist = sqrt(point[0]*point[0] + point[1]*point[1])
				self.controller.MoveDistance(distance)
				#Random choice +/-90 
				#self.controller.RotateByTheta(theta)

			self.Align(frame)


	