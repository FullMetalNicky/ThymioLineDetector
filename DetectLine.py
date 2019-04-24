import cv2
import numpy as np
import os

from LineDetector import LineDetector
from ThymioCamera import ThymioCamera

k = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
d = np.array([0.0,0.0,0.0,0.0,0.0])
cam = ThymioCamera(640, 480,k, d)
ld = LineDetector(cam)
folderPath = "images/"
files = os.listdir(folderPath)
files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
for file in files:
    if file.endswith(".png"):
    	frame = cv2.imread(folderPath+file)
    	ld.Flow(frame)
    	key = cv2.waitKey(10)
    	if(key > 0):
    		break
	