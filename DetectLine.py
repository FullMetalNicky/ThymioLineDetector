import cv2
import numpy as np
import os

from LineDetector import LineDetector

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
	