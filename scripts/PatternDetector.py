import cv2
import numpy as np

from MazePatterns import MazePatterns

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
		self.patternDictionary[MazePatterns.destination] = destination

		cross = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		cross[self.gridRowNumber/2, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		cross[:, self.gridColNumber/2] = np.ones([self.gridRowNumber], dtype = "uint8")
		self.patternDictionary[MazePatterns.crossroads] = cross

		tjunc = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		tjunc[self.gridRowNumber/2, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		tjunc[self.gridRowNumber/2: self.gridRowNumber, self.gridColNumber/2] = np.ones([self.gridRowNumber - self.gridRowNumber/2], dtype = "uint8")
		self.patternDictionary[MazePatterns.regTJunction] = tjunc


		righttjunc = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		righttjunc[:, self.gridColNumber/2] = np.transpose(np.ones([self.gridRowNumber], dtype = "uint8"))
		righttjunc[self.gridRowNumber/2, self.gridColNumber/2:self.gridColNumber] = np.ones([1, self.gridColNumber/2 +1], dtype = "uint8")
		self.patternDictionary[MazePatterns.rightTjunction] = righttjunc
	#	print("righttjunc")
	#	print(righttjunc)

		leftTjunction = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		leftTjunction[:, self.gridColNumber/2] = np.transpose(np.ones([self.gridRowNumber], dtype = "uint8"))
		leftTjunction[self.gridRowNumber/2, 0:(self.gridColNumber/2 + 1)] = np.ones([1, self.gridColNumber/2+1], dtype = "uint8")
		self.patternDictionary[MazePatterns.leftTjunction] = leftTjunction
	#	print("leftTjunction")
	#	print(leftTjunction)

		rightturn= np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		rightturn[self.gridRowNumber/2, self.gridColNumber/2:self.gridColNumber] = np.ones([1, self.gridColNumber/2+1], dtype = "uint8")
		rightturn[self.gridRowNumber/2: self.gridRowNumber, self.gridColNumber/2] = np.ones([self.gridRowNumber - self.gridRowNumber/2], dtype = "uint8")
		self.patternDictionary[MazePatterns.rightturn] = rightturn
	#	print("rightturn")
	#	print(rightturn)

		leftturn= np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		leftturn[self.gridRowNumber/2, 0:(self.gridColNumber/2 + 1)] = np.ones([1, self.gridColNumber/2+1], dtype = "uint8")
		leftturn[self.gridRowNumber/2: self.gridRowNumber, self.gridColNumber/2] = np.ones([self.gridRowNumber - self.gridRowNumber/2], dtype = "uint8")
		self.patternDictionary[MazePatterns.leftturn] = leftturn
	#	print("leftturn")
	#	print(leftturn)

		ortholine = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		ortholine[self.gridRowNumber/2, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		self.patternDictionary[MazePatterns.ortholine] = ortholine

		paraline = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		paraline[:, self.gridColNumber/2] = np.transpose(np.ones([self.gridRowNumber], dtype = "uint8"))
		self.patternDictionary[MazePatterns.paraline] = paraline
		
		deadend = np.zeros([self.gridRowNumber, self.gridColNumber], dtype = "uint8")
		deadend[self.gridRowNumber - 1, :] = np.ones([1, self.gridColNumber], dtype = "uint8")
		self.patternDictionary[MazePatterns.deadend] = deadend



	def SetImageSize(self, imageWidth, imageHeight):
		self.imageWidth = imageWidth
		self.imageHeight = imageHeight
		self.gridWidth = int(imageWidth / self.gridColNumber)
		self.gridHeight = int(imageHeight / self.gridRowNumber)
		self.th = int(self.gridWidth*self.gridHeight*0.2)
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

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.destination])
		if tmp.sum() == self.gridRowNumber * self.gridColNumber:
			return MazePatterns.destination

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.crossroads])
		if tmp.sum() == self.gridRowNumber + self.gridColNumber - 1:
			return MazePatterns.crossroads

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.regTJunction])
		if tmp.sum() == (self.gridRowNumber/2 + self.gridColNumber):
			return MazePatterns.regTJunction

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.rightTjunction])
		if tmp.sum() == (self.gridRowNumber + self.gridColNumber/2):
			return MazePatterns.rightTjunction

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.leftTjunction])
		if tmp.sum() == (self.gridRowNumber + self.gridColNumber/2):
			return MazePatterns.leftTjunction

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.rightturn])
		if tmp.sum() == (self.gridRowNumber/2 + self.gridColNumber/2 + 1):
			return MazePatterns.rightturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.leftturn])
		if tmp.sum() == (self.gridRowNumber/2 + self.gridColNumber/2 + 1):
			return MazePatterns.leftturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.ortholine])
		if tmp.sum() == self.gridColNumber: 
			return MazePatterns.ortholine

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.paraline])
		if tmp.sum() > self.gridRowNumber/2:
			return MazePatterns.paraline
		#tmp = np.multiply(frame, self.patternDictionary[MazePatterns.deadend])
		if tmp.sum() > 0:
			return MazePatterns.deadend	

		if frame.sum() == 0:
			return MazePatterns.noline

		return MazePatterns.weird