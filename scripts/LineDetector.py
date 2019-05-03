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
		self.processedFrames = []
		self.video_publisher = rospy.Publisher(self.thymio_name + '/camera/video', Image, queue_size=10)
		self.camera_info_subscriber = rospy.Subscriber(self.thymio_name + '/camera/camera_info',CameraInfo, self.update_camera_info)
		self.tf_listener = tf.TransformListener()
		self.state = "none"
		self.pinhole_camera = PinholeCameraModel()
		self.state_coords = []
		self.center_coords = []

	def update_camera_info(self, data):
		self.image_width = data.width
		self.image_height = data.height
		self.pinhole_camera.fromCameraInfo(data)
		self.init_camera = 1
		self.camera_info_subscriber.unregister()

	def GetImageSize(self):
		return [self.image_width, self.image_height ]

	def GetPatternState(self):
		return self.state	

	def GetStateCoords(self):
		return self.state_coords

	def GetTopViewFrame(self):
		if len(self.processedFrames) > 0:
			return self.processedFrames[len(self.processedFrames) - 1]
		return None

	def update_camera_stream(self, data):
		frame = self.bridge.imgmsg_to_cv2(data)
		processedFrame = self.processFrame(frame)
		
		self.processedFrames.append(processedFrame)
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
		self.center_coords = self.Get3DRobotCoordsFromImage(self.image_width/2, self.image_height/2)
		self.camera_subscriber = rospy.Subscriber(self.thymio_name + '/camera/image_raw',Image, self.update_camera_stream)

	def SetPose(self, trans, rot):
		self.trans= trans
		self.rot = rot
		self.ComputeRobotImageTransform()

	def GetCenter3DCoords(self):
		return self.center_coords

	def Get3DRobotCoordsFromImage(self, u, v):
		[a,b,c] = self.pinhole_camera.projectPixelTo3dRay([u, v])
		#cam->robot x->-y', y->-z', z->x'
		camHomVec = np.transpose(np.array([c,-a,-b,1]))
		R = self.GetRotationMatrix(self.rot[0], self.rot[1], self.rot[2])
		robotHomVec = np.dot(R,camHomVec)
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

	def TransformTopToImage(self, u, v):
		homVec = np.transpose(np.array([[u, v, 1]], dtype=np.float32))
		newVec = np.dot(self.H, homVec)
		newVec = newVec/newVec[2]
		return [newVec[0], newVec[1]]


	def ComputeRobotImageTransform(self):
		R = self.GetRotationMatrix(self.rot[0], self.rot[1], self.rot[2])
		T = translation_matrix(self.trans)
		F = concatenate_matrices(T,R)
		self.robot_image_transform = F

	def ComputeHomography(self):
		K = self.pinhole_camera.intrinsicMatrix()
		R = self.GetRotationMatrix(self.rot[1] - PI/2, self.rot[0] , self.rot[2] )
		A1 = np.array([ [1, 0, -self.image_width/2], [0, 1, -self.image_height/2], [0, 0, 0], [ 0, 0, 1 ]])
		T =  np.array([ [1, 0, 0, 0],  [0, 1, 0, 0],  [0, 0, 1, K[0,0]], [0, 0, 0, 1]])
		Kr = np.array([ [K[0,0], 0, self.image_width/2,0], [0, K[1,1], self.image_height/2,0 ], [ 0, 0, 1, 0]])
		H = np.dot(Kr ,np.dot(T, np.dot(R,A1)))
		self.H = H
		self.dsize = (self.image_width,self.image_height)
	

	def LineMaskByColor(self,frame):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lowTH = np.array([0,0,0], np.uint8)
		highTH = np.array([180,255,30], np.uint8)
		mask = cv2.inRange(hsv, lowTH, highTH)
		return mask

	def GetNonZero(self, points):
		frame = frame.astype(dtype="uint8")
		nonZ = cv2.findNonZero(frame)
		return nonZ

	def GetLineInRobotFrame(self, frame):
		frame = frame.astype(dtype="uint8")
		nonZ = cv2.findNonZero(frame)
		line = cv2.fitLine(nonZ, cv2.DIST_L2, 0, 0.01, 0.01)
		px1 = line[2]
		py1 = line[3]
		px2 = px1 + 2*line[0]
		py2 = py1 + 2*line[1]
		[u1, v1] = self.TransformTopToImage(px1, py1)
		[u2, v2] = self.TransformTopToImage(px2, py2)
		l1 = self.Get3DRobotCoordsFromImage(u1, v1)
		l1[2] = 0
		l2 = self.Get3DRobotCoordsFromImage(u2, v2)
		direction = [l2[0] - l1[0], l2[1] - l1[1]]
		direction  = direction / np.sqrt(direction[0]*direction[0] + direction[1]*direction[1])
		return l1, direction


	def IsLineStraight(self, frame):
		frame = frame.astype(dtype="uint8")
		eps = 0.1
		nonZ = cv2.findNonZero(frame)
		line = cv2.fitLine(nonZ, cv2.DIST_L2, 0, 0.01, 0.01)
		if (np.fabs(line[0]) < eps) and (1.0 - np.fabs(line[1]) < eps):
			return True
		return False

	def GetLineOffset(self, frame):
		frame = frame.astype(dtype="uint8")
		nonZ = cv2.findNonZero(frame)
		x,y,w,h = cv2.boundingRect(nonZ)
		boxCenter = (x + x + w)/2
		p1 = self.Get3DRobotCoordsFromImage(boxCenter, 0)
		p2 = self.Get3DRobotCoordsFromImage(self.image_width/2, 0)
		return p2[1] - p1[1]

		
	def TopView(self,frame):
		top = cv2.warpPerspective(frame, self.H, self.dsize, flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
		return top	


	def ConcatImages(self,img1, img2):
		if 3 == len(img1.shape):
			img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
		if 3 == len(img2.shape):
			img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

		h1, w1 = img1.shape[:2]	
		h2, w2 = img2.shape[:2]	
		h = max(h1, h2)
		w = w1 +w2
		combined = np.zeros([h, w], dtype = "uint8")
		combined[0:h, 0:w1] = img1
		combined[0:h, w1:w1+w2] = img2
		return combined

	def processFrame(self, frame):
		hsvmask = self.LineMaskByColor(frame)
	 	top = self.TopView(hsvmask)
	 	return top


