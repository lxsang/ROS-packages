#! /usr/bin/python
import cv2
import sys
import os
import ntpath

f = sys.argv[1]
print('Binarizing ',f)
name = ntpath.basename(f)
dir =  os.path.dirname(f)
im_gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
#THRESH_BINARY_INV
if(len(sys.argv) > 2):
    thresh = 130
    im_bw = cv2.threshold(im_gray, thresh, 255, cv2.THRESH_BINARY_INV)[1]
else:    
    (thresh, im_bw) = cv2.threshold(im_gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
cv2.imwrite(dir + '/bin_' + name, im_bw)