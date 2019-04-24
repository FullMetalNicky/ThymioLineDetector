import cv2
import numpy as np
import os

class LineDetector:
	def __init__(self, width, height):
		self.image_width = width
		self.image_height = height

	def LineMaskByColor(self,frame):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lowTH = np.array([0,0,0], np.uint8)
		highTH = np.array([180,255,30], np.uint8)
		mask = cv2.inRange(hsv, lowTH, highTH)
		return mask
		
	def LineMaskByEdge(self,frame):
		grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
		sharp = cv2.filter2D(grey, -1, kernel)
		edges = cv2.Canny(sharp, 20, 200)
		#mask = cv2.threshold(edges, 0, 255, cv2.THRESH_BINARY)
		return sharp

	def TopView(self,frame):
		maxWidth = self.image_width-1;
		maxHeight = self.image_height-1
		src = np.array([[270,100], [370,100], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
		dst = np.array([[0,0], [maxWidth,0], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
		H = cv2.getPerspectiveTransform(src, dst)
		top = cv2.warpPerspective(frame, H, frame.shape[:2])
		return top	

	def DetectLines(self,frame):
		threshold =60
		lines = cv2.HoughLinesP(frame, 1, np.pi/180, threshold, 0, 10, 20);	
		return lines

	def ExtractLineBlocks2(self,frame, lines):
		lineimg = np.copy(frame)
		lineimg = cv2.cvtColor(lineimg, cv2.COLOR_GRAY2BGR)
		points = []
		for line in lines:
			x1, y1, x2, y2 = line[0]
			points.append([x1, y1])
			points.append([x2, y2])
		points = np.array(points)
		x, y, w, h = cv2.boundingRect(points)	
		print(x, y, w, h)
		cv2.rectangle(lineimg, (x, y), (x+w, y+h), (0, 255, 0), 2)
		return lineimg

	def ExtractConnectedComponents(self,frame):
		frame = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)[1]
		retval, labels = cv2.connectedComponents(frame)
		label_hue = np.uint8(179*labels/np.max(labels))
		blank_ch = 255*np.ones_like(label_hue)
		labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])
		labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)
		labeled_img[label_hue==0] = 0
		return labeled_img

	def ExtractLineBlocks(self,frame):
		frame = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)[1]
		contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		#im, contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		ctrimg = np.zeros(frame.shape, dtype = "uint8")
		ctrimg = cv2.cvtColor(ctrimg, cv2.COLOR_GRAY2BGR)
		cv2.drawContours(ctrimg, contours, -1, (0,255,0), 3)
		return ctrimg


	def CreateLineMask(self,frame, lines):
		lineimg = np.zeros(frame.shape, dtype = "uint8")
		for line in lines:
			for x1,y1,x2,y2 in line:
				cv2.line(lineimg,(x1,y1),(x2,y2),(255),10)  
		return lineimg
	 
	def DrawLines(self,frame, lines):
	 	lineimg = np.zeros(frame.shape, dtype = "uint8")
	 	lineimg = cv2.cvtColor(lineimg, cv2.COLOR_GRAY2BGR)
	 	for line in lines:
	 		for x1,y1,x2,y2 in line:
	 			cv2.line(lineimg,(x1,y1),(x2,y2),(255, 0, 0),10)  
	 	return lineimg

	def Flow(self, frame):
	 	hsvmask = self.LineMaskByColor(frame)
	 	top = self.TopView(hsvmask)
 		lines = self.DetectLines(top)
 		line = self.CreateLineMask(top, lines)
 		block = self.ExtractLineBlocks(line)
 		cv2.imshow("frame", frame)
 		cv2.imshow("block", block)

ld = LineDetector(640, 480)
folderPath = "images/"
files = os.listdir(folderPath)
files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
for file in files:
    if file.endswith(".png"):
    	frame = cv2.imread(folderPath+file)
    	ld.Flow(frame)
    	key = cv2.waitKey(100)
    	if(key > 0):
    		break
	