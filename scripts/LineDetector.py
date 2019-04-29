import cv2
import numpy as np
from ThymioCamera import ThymioCamera
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from PatternDetector import PatternDetector


PI=3.1415926

class LineDetector:
	def __init__(self, name):
		self.thymio_name = name
		self.bridge = CvBridge()
		self.init_tf=0
		self.init_camera = 0
		self.processedFrame = []
		self.video_publisher = rospy.Publisher(self.thymio_name + '/camera/video', Image, queue_size=10)
		self.camera_info_subscriber = rospy.Subscriber(self.thymio_name + '/camera/camera_info',CameraInfo, self.update_camera_info)
		self.tf_listener = tf.TransformListener()
		self.pattern_detector = PatternDetector(5,3)
		self.state = "none"

	def update_camera_info(self, data):
		camera = ThymioCamera(data.width, data.height, data.K, data.D)
		self.SetCamera(camera)
		self.pattern_detector.SetImageSize(data.width, data.height)
		self.init_camera = 1
		self.camera_info_subscriber.unregister()

	def update_camera_stream(self, data):
		frame = self.bridge.imgmsg_to_cv2(data)
		frame= np.uint8(frame)
		processedFrame = self.processFrame(frame)
		patternMat = self.pattern_detector.CreatePatternMatrix(processedFrame)
		state = self.pattern_detector.GetPattern(patternMat)
		if (state != self.state):
			self.state = state
			print(state)
		#self.processedFrame = processedFrame
		msg = self.bridge.cv2_to_imgmsg(processedFrame)
		self.video_publisher.publish(msg)	

	def Init(self):
		while self.init_tf == 0:
		    try:
		        (trans,rot)=self.tf_listener.lookupTransform(self.thymio_name +'/base_link', self.thymio_name +'/camera_link', rospy.Time(0))
		    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		        continue
		    self.SetPose(trans, euler_from_quaternion(rot))
		    self.init_tf=1

		while (self.init_tf == 0) or (self.init_camera ==0):
		    continue

		self.ComputeHomography()
		self.ComputeFakeHomography()
		self.camera_subscriber = rospy.Subscriber(self.thymio_name + '/camera/image_raw',Image, self.update_camera_stream)

	def SetCamera(self, camera):
		self.camera = camera

	def SetPose(self, trans, rot):
		self.trans= trans
		self.rot = rot

	def ComputeHomography(self):

		k = self.camera.GetCameraK()
		k = np.reshape(k, (3, 3))
		d = self.camera.GetCameraD()
		w, h = self.camera.GetCameraSize()
		thymioHeight = self.trans[2]
		thymioYOffset = self.trans[1]
		thymioXOffset = self.trans[0]
		roll = self.rot[0]
		pitch = self.rot[1] 
		yaw = self.rot[2]
		scale = 100
		M = np.transpose(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [thymioXOffset, thymioYOffset, -thymioHeight]]))
		S =  np.array([[scale, 0.0, 0.0], [0.0, scale, 0.0], [0, 0, 1]])
		M = np.matmul(M, S)
		size = (h, w)
		#self.mapx, self.mapy = cv2.initUndistortRectifyMap(k, d, np.linalg.inv(M), k, size, cv2.CV_32FC1) 
		self.mapx, self.mapy = cv2.initUndistortRectifyMap(k, d, M, k, size, cv2.CV_32FC1) 

	def ComputeFakeHomography(self):
		width, height = self.camera.GetCameraSize()
		maxWidth = width-1;
		maxHeight = height-1
		src = np.array([[270,100], [370,100], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
		dst = np.array([[0,0], [maxWidth,0], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
		H = cv2.getPerspectiveTransform(src, dst)
		#print(H)
		self.H = H

	def LineMaskByColor(self,frame):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lowTH = np.array([0,0,0], np.uint8)
		highTH = np.array([180,255,30], np.uint8)
		mask = cv2.inRange(hsv, lowTH, highTH)
		return mask
		
	def TopView(self,frame):
		width, height = self.camera.GetCameraSize()
		top = cv2.warpPerspective(frame, self.H, (width, height))
		#top = cv2.warpPerspective(frame, self.H, (width, height), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
		#top = cv2.remap(frame, self.mapx, self.mapy, interpolation=cv2.INTER_LINEAR)
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

	def ExtractLineBlocks(self,frame):
		frame = cv2.threshold(frame, 127, 255, cv2.THRESH_BINARY)[1]
		#contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		im, contours, hierarchy = cv2.findContours(frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		ctrimg = np.zeros(frame.shape, dtype = "uint8")
		ctrimg = cv2.cvtColor(ctrimg, cv2.COLOR_GRAY2BGR)
		cv2.drawContours(ctrimg, contours, -1, (0,255,0), 3)
		return ctrimg , contours

	def GetLineCoordinates(self, contours):
		rects=[]
		for c in contours:
			area = cv2.contourArea(c)
			if area > 200:
				x,y,w,h = cv2.boundingRect(c)
				rects.append((x,y,w,h))
		return rects

	def DrawRects(self, frame, rects):
		if 3 > len(frame.shape):
			frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
		for rect in rects:
			x, y, w, h = rect
			cv2.rectangle(frame, (x, y), (x+w, y+h), (2 * x, 4 *y, 50), 2)
		return frame

	def CreateLineMask(self,frame, lines):
		lineimg = np.zeros(frame.shape, dtype = "uint8")
		for line in lines:
			for x1,y1,x2,y2 in line:
				cv2.line(lineimg,(x1,y1),(x2,y2),(255),10)  
		return lineimg

	def ConcatImages(self,img1, img2):
		h1, w1 = img1.shape[:2]	
		h2, w2 = img2.shape[:2]	
		h = max(h1, h2)
		w = w1 +w2
		combined = np.zeros([h, w], dtype = "uint8")
		img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
		combined[0:h, 0:w1] = img1
		combined[0:h, w1:w1+w2] = img2
		return combined

	def processFrame(self, frame):
		hsvmask = self.LineMaskByColor(frame)
	 	top = self.TopView(hsvmask)
	 	return top

	def Flow(self, frame):
	 	hsvmask = self.LineMaskByColor(frame)
	 	top = self.TopView(hsvmask)
 		#lines = self.DetectLines(top)
 		#line = self.CreateLineMask(top, lines)
 		#block, contours = self.ExtractLineBlocks(line)
 		#rects = self.GetLineCoordinates(contours)
 		#rect = self.DrawRects(top, rects)
 		#cv2.imshow("frame", frame)
 		#cv2.imshow("rect", rect)
 		#cv2.imshow("top", top)
 		combined = self.ConcatImages(frame, top)
 		return combined
