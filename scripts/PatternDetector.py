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
		destination = np.array([[1,1,1], [1,1,1], [1,1,1], [1,1,1], [1,1,1]], dtype = "float32")
		self.patternDictionary[MazePatterns.destination] = destination

		cross = np.array([[0,1,0], [0,1,0], [1,1,1], [0,1,0], [0,1,0]], dtype = "float32")
		self.patternDictionary[MazePatterns.crossroads] = cross

	#	tjunc = np.array([[-1,-1,-1], [-1,-1,-1], [1,1,1], [0,1,0], [0,1,0]], dtype = "uint8")
	#	self.patternDictionary[MazePatterns.regTJunction] = tjunc

	
		rightturn= np.array([[-1,-1,-1], [-1,-1,-1], [-1,1,1], [0,1,0], [1,1,0]], dtype = "float32")
		self.patternDictionary[MazePatterns.rightturn] = rightturn
		#print(rightturn)

		leftturn= np.array([[-1,-1,-1], [-1,-1,-1], [1,1,-1], [0,1,0], [0,1,1]], dtype = "float32")
		self.patternDictionary[MazePatterns.leftturn] = leftturn
#		print(leftturn)

		ortholine =  np.array([[-1,-1,-1], [-1,-1,-1], [1,1,1], [0,1,0], [-1,1,-1]], dtype = "float32")
		self.patternDictionary[MazePatterns.ortholine] = ortholine

		paraline = np.array([[0,1,0], [0,1,0], [0,1,0], [0,1,0], [0,1,0]], dtype = "float32")
		self.patternDictionary[MazePatterns.paraline] = paraline
		
		deadend = np.array([[0,0,0], [0,0,0], [-1,0,-1], [-1,1,-1], [-1,1,-1]], dtype = "float32")
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
		if tmp.sum() >= 15:
			return MazePatterns.destination

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.crossroads])
		if tmp.sum() == self.gridRowNumber + self.gridColNumber - 1:
			return MazePatterns.crossroads

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.rightturn])
		if tmp.sum() >= 5:
			return MazePatterns.rightturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.leftturn])
		if tmp.sum() >= 5:
			return MazePatterns.leftturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.ortholine])
		if tmp.sum() == 5: 
			return MazePatterns.ortholine

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.rightturn])
		if tmp.sum() >= 4: 
			return MazePatterns.rightturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.leftturn])
		if tmp.sum() >= 4: 
			return MazePatterns.leftturn

		tmp = np.multiply(frame, self.patternDictionary[MazePatterns.paraline])
		if tmp.sum() > 2:
			return MazePatterns.paraline
		#tmp = np.multiply(frame, self.patternDictionary[MazePatterns.deadend])
		if tmp.sum() > 0:
			return MazePatterns.deadend	

		if frame.sum() == 0:
			return MazePatterns.noline

		return MazePatterns.weird