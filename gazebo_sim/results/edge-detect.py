#! /usr/bin/python

import cv2
import numpy as np
import sys
import os
import ntpath

from matplotlib import pyplot as plt

name = ntpath.basename(sys.argv[1])
dir =  os.path.dirname(sys.argv[1])

img = cv2.imread(sys.argv[1],0)
edges = 255 - cv2.Canny(img,100,200)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()


cv2.imwrite(dir + "/edge_" + name, edges)