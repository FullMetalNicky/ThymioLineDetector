import cv2
import numpy as np

class ThymioCamera:
	def __init__(self, width, height, k, d):
		self.width = width
		self.height = height
		self.k = k
		self.d = d

	def GetCameraSize(self):
		return self.width, self.height

	def UndistortImage(self, frame):
		return cv2.undistort(frame, self.k, self.d)

	def UndistortPoints(self, points):
		return cv2.undistort(points, self.k, self.d)