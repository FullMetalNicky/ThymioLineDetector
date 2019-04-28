import cv2
import numpy as np
from PatternDetector import PatternDetector

frame = cv2.imread("images/0.png")
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
lowTH = np.array([0,0,0], np.uint8)
highTH = np.array([180,255,30], np.uint8)
mask = cv2.inRange(hsv, lowTH, highTH)

maxWidth = 640-1;
maxHeight = 480-1
src = np.array([[270,100], [370,100], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
dst = np.array([[0,0], [maxWidth,0], [maxWidth,maxHeight], [0,maxHeight]], dtype = "float32")
H = cv2.getPerspectiveTransform(src, dst)
#print(H)
#M = np.array([[1.0, 0.0, 0.028], [0.0, 1.0, 0.009], [0.0, 0.0, 1.0]])
top = cv2.warpPerspective(mask, H, (640, 480), cv2.WARP_INVERSE_MAP)


pd = PatternDetector(3,5)
pd.SetImageSize(640, 480)
pd.InitGridMasks()
patternMat = pd.CreatePatternMatrix(top)
print(patternMat)
cv2.imshow("top", top)
cv2.waitKey()