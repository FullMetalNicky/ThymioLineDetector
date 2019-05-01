import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from PatternDetector import PatternDetector
from numpy.linalg import inv
from image_geometry import PinholeCameraModel 
from  tf.transformations import rotation_matrix, concatenate_matrices, translation_matrix

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
		self.pinhole_camera = PinholeCameraModel()
		self.state_coords = []

	def update_camera_info(self, data):
		self.image_width = data.width
		self.image_height = data.height
		self.pinhole_camera.fromCameraInfo(data)
		self.init_camera = 1
		self.camera_info_subscriber.unregister()

	def GetPatternState(self):
		return self.state	
		
	def GetStateCoords(self):
		return self.state_coords

	def update_camera_stream(self, data):
		frame = self.bridge.imgmsg_to_cv2(data)
		frame= np.uint8(frame)
		processedFrame, cropFrame = self.processFrame(frame)
		patternMat = self.pattern_detector.CreatePatternMatrix(cropFrame)
		state = self.pattern_detector.GetPattern(patternMat)

		if(state != "noline"):
			[u, v] = self.CalcCenterOfMass(processedFrame)
			cv2.circle(frame, (u,v), 5, (255, 255, 255), -1)
			[x,y,z] = self.Get3DRobotCoordsFromImage(u, v)
			self.state_coords = [x,y,z]	
			if (state != self.state):
				[a,b,c] = self.pinhole_camera.projectPixelTo3dRay([u, v])
				camHomVec = np.transpose(np.array([c,-a,-b,1]))
				robotHomVec = np.dot(self.robot_image_transform,camHomVec)
				print(u, v)
				print(x, y, z)
				print(state)
	
		self.state = state			
			
		self.processedFrame = processedFrame
		comb = self.ConcatImages(frame, processedFrame)
		msg = self.bridge.cv2_to_imgmsg(comb)
		self.video_publisher.publish(msg)	

	def GetRotationMatrix(self, alpha, beta, gamma):
		origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
		Rx = rotation_matrix(alpha, xaxis)
		Ry = rotation_matrix(beta, yaxis)
		Rz = rotation_matrix(gamma, zaxis)
		R = concatenate_matrices(Rx, Ry, Rz)
		return R

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
		self.pattern_detector.SetImageSize(self.image_width - 2*self.cropX, self.image_height)
		self.camera_subscriber = rospy.Subscriber(self.thymio_name + '/camera/image_raw',Image, self.update_camera_stream)

	def SetPose(self, trans, rot):
		self.trans= trans
		self.rot = rot
		self.ComputeRobotImageTransform()

	def Get3DRobotCoordsFromImage(self, u, v):
		[a,b,c] = self.pinhole_camera.projectPixelTo3dRay([u, v])
		#cam->robot x->-y', y->-z', z->x'
		camHomVec = np.transpose(np.array([c,-a,-b,1]))
		robotHomVec = np.dot(self.robot_image_transform,camHomVec)

		#The transformation is from the base link to the camera, but does not include the base link height
		#so we need to subtract it, if we want z=0 to actually be the ground plane
		baseHeight = 0.053
		[x,y,z] = self.trans
		alpha = (baseHeight-z)/robotHomVec[2]
		return [x+alpha*robotHomVec[0], y+alpha*robotHomVec[1], z +alpha*robotHomVec[2]]

	def CalcCenterOfMass(self, frame):
		frame = frame.astype(dtype="uint8")
		nonZ = np.array(cv2.findNonZero(frame))
		mx = nonZ[:,:, 0].mean()
		my = nonZ[:, :, 1].mean()
		homVec = np.transpose(np.array([[mx, my, 1]], dtype=np.float32))
		newVec = np.dot(self.H, homVec)
		newVec = newVec/newVec[2]
		return [newVec[0], newVec[1]]

	def ComputeRobotImageTransform(self):
		R = self.GetRotationMatrix(self.rot[0], self.rot[1], self.rot[2])
		T = translation_matrix(self.trans)
		F = concatenate_matrices(T,R)
		self.robot_image_transform = R 

	def ComputeHomography(self):
		K = self.pinhole_camera.intrinsicMatrix()
		w = self.image_width
		h = self.image_height
		roll = self.rot[0] 
		pitch = self.rot[1] - PI/2
		yaw = self.rot[2] 
		R = self.GetRotationMatrix(pitch, roll, yaw)
		A1 = np.array([ [1, 0, -w/2], [0, 1, -h/2], [0, 0, 0], [ 0, 0, 1 ]])
		T =  np.array([ [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, K[0,0]], [0, 0, 0, 1]])
		Kr = np.array([ [K[0,0], 0, w/2,0], [0, K[1,1], h/2,0 ], [ 0, 0, 1, 0]])
		H = np.dot(Kr ,np.dot(T, np.dot(R,A1)))
		self.H = H
		self.dsize = (w,h)
		self.cropX = int(h/2*(np.tan(self.rot[1])))
	

	def LineMaskByColor(self,frame):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lowTH = np.array([0,0,0], np.uint8)
		highTH = np.array([180,255,30], np.uint8)
		mask = cv2.inRange(hsv, lowTH, highTH)
		return mask
		
	def TopView(self,frame):
		top = cv2.warpPerspective(frame, self.H, self.dsize, flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
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
	 	croptop = top[:, self.cropX:(self.image_width- self.cropX)]
	 	return top, croptop

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
