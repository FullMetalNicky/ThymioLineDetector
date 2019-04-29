import cv2
import numpy as np


class PatternDetector:
	def __init__(self, gridRowNumber, gridColNumber):
		self.gridRowNumber = gridRowNumber
		self.gridColNumber = gridColNumber
		self.masks = []
		self.imageWidth = 0
		self.imageHeight = 0
		self.gridWidth = 0
		self.gridHeight = 0
		self.th = 0
		self.patternDictionary = dict()

	def InitStateMasks(self):
		destination = np.ones([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		self.patternDictionary["destination"] = destination

		cross = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		cross[self.gridRowNumber - 1, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		cross[:, self.gridColNumber/2] = np.ones([self.gridRowNumber], dtype = "uint8")
		self.patternDictionary["cross"] = cross
		
		paraline = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		paraline[:, self.gridColNumber/2] = np.transpose(np.ones([self.gridRowNumber], dtype = "uint8"))
		self.patternDictionary["paraline"] = paraline
		
		deadend = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		deadend[self.gridRowNumber - 1, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		self.patternDictionary["deadend"] = deadend



	def SetImageSize(self, imageWidth, imageHeight):
		self.imageWidth = imageWidth
		self.imageHeight = imageHeight
		self.gridWidth = int(imageWidth / self.gridColNumber)
		self.gridHeight = int(imageHeight / self.gridRowNumber)
		self.th = int(self.gridWidth*self.gridWidth*0.05)
		self.InitGridMasks()
		self.InitStateMasks()

	def InitGridMasks(self):
		onesMat = np.ones([self.gridHeight, self.gridWidth], dtype = "uint8")
		for c in range(0,self.gridColNumber):
			for r in range (0, self.gridRowNumber):
				mask = np.zeros([self.imageHeight, self.imageWidth], dtype = "uint8");
				x = c * self.gridWidth;
				y = r * self.gridHeight;
				mask[y:y+self.gridHeight, x:x+self.gridWidth] = onesMat
				self.masks.append(mask)

	def ApplyMask(self, frame, ind):
		res = frame * self.masks[ind];
		nonZ = cv2.findNonZero(res)
		#print(nonZ)
		if nonZ is not None:
			if nonZ.shape[0]>self.th:
				return 1;
		return 0;

	def CreatePatternMatrix(self, frame):
		patternMat = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8");
		for c in range(0,self.gridColNumber):
			for r in range (0, self.gridRowNumber):
				ind = c*self.gridRowNumber + r;
				patternMat[r, c]= self.ApplyMask(frame, ind)
		return patternMat


	def GetPattern(self, frame):

		tmp = np.multiply(frame, self.patternDictionary["destination"])
		if tmp.sum() == self.gridRowNumber * self.gridColNumber:
			return "destination"

		if tmp.sum() == self.gridColNumber or tmp.sum() == 2 * self.gridColNumber:
			return "ortholine"

		tmp = np.multiply(frame, self.patternDictionary["cross"])
		if tmp.sum() >= self.gridRowNumber + self.gridColNumber - 1:
			return "cross"

		tmp = np.multiply(frame, self.patternDictionary["deadend"])
		if tmp.sum() == 1:
			return "deadend"

		tmp = np.multiply(frame, self.patternDictionary["paraline"])
		if tmp.sum() > 1:
			return "paraline"

		if frame.sum() == 0:
			return "noline"

		return "weird"