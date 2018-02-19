#! /usr/bin/python

import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
from skimage.measure import structural_similarity as ssim


# using normalize neareast distance of all pixel
def nearest_error(groundtruth, slammap):
    cnt = 0
    sl_occupancies = None
    # build the training set from the aligned slam map
    for i in range(slammap.shape[0]):
        for j in range(slammap.shape[1]):
            pixel = slammap.item(i, j)
            if(pixel < 50):
                if(sl_occupancies is None):
                    sl_occupancies = [i,j]
                else:
                    sl_occupancies = np.vstack((sl_occupancies, [i,j]))

    #print sl_occupancies[2]
    #for x in np.nditer(sl_occupancies):
    #    print x,"\n"

    sl_occupancies = sl_occupancies.astype(np.float32)
    # label them
    gr_labels = np.full((sl_occupancies.shape[0], 1), 1).astype(np.float32)


    # Feature set containing (x,y) values of 25 known/training data
    #trainData = np.random.randint(0,100,(25,2)).astype(np.float32)

    # Labels each one either Red or Blue with numbers 0 and 1
    #responses = np.random.randint(0,2,(25,1)).astype(np.float32)

    # plot Reds
    #x, y = sl_occupancies.T
    #plt.scatter(x,y)
    #red = sl_occupancies[gr_labels.ravel()==1]
    #plt.scatter(red[:,0],red[:,1],80,'r','^')

    # plot Blues
    #blue = trainData[responses.ravel()==1]
    #plt.scatter(blue[:,0],blue[:,1],80,'b','s')

    # CvKNearest instance
    knn = cv2.ml.KNearest_create()

    # trains the model
    knn.train(sl_occupancies, cv2.ml.ROW_SAMPLE,gr_labels)

    # New sample : (x,y)
    #newcomer = np.array([[500,500],[100,100]]).astype(np.float32)
    #plt.scatter(newcomer[:,0],newcomer[:,1],80,'g','o')

    # find nearest cell on the slam map for each occupancy cell in the groundtruth map
    gr_occupancies = None
    ncells = 0
    for i in range(groundtruth.shape[0]):
        for j in range(groundtruth.shape[1]):
            pixel = groundtruth.item(i, j)
            if(pixel < 50):
                ncells = ncells+1
                if(gr_occupancies is None):
                    gr_occupancies = [i,j]
                else:
                    gr_occupancies = np.vstack((gr_occupancies, [i,j]))
    gr_occupancies = gr_occupancies.astype(np.float32)
    ret, results, neighbours, dist = knn.findNearest(gr_occupancies, 1)

    #cv2.imshow("Different", cv2.absdiff(groundtruth,slammap))

    # print the result
    #print "ret: ", ret, "\n"
    #print "result: ", results,"\n"
    #print "neighbours: ", neighbours,"\n"
    #print "distance: ", dist
    sume = np.sum(dist)
    print "sum of error: ", sume, "\n"
    print "normalize error:", (sume/ncells), "\n"
    #plt.show()
    #cv2.waitKey(0)


# using mean square error
def mse(imageA, imageB):
    # the 'Mean Squared Error' between the two images is the
    # sum of the squared difference between the two images;
    # NOTE: the two images must have the same dimension
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])

    # return the MSE, the lower the error, the more "similar"
    # the two images are
    print "mean square error:", err 
    return err

# using structure similarity index
def issim(imageA,imageB):
    e = ssim(imageA,imageB)
    print "Structure similarity idex: ", e

############################MAIN PROGRAM###################################
# I. FIRST STEP: IMAGE REGISTRATION
# the SLAM map will be aligned to the groundtruth map using ECC algorithm

# Read the images to be aligned as grayscale
groundtruth =  cv2.imread(sys.argv[1],0)
slammap_unaligned =  cv2.imread(sys.argv[2],0)

align = False
method = 2 # nearest
# Convert images to grayscale
#im1_gray = cv2.cvtColor(im1,cv2.COLOR_BGR2GRAY)
#im2_gray = cv2.cvtColor(im2,cv2.COLOR_BGR2GRAY)
if align is True: 
    print 'Align images', "\n"
    # Find size of the groundtruth
    sz = groundtruth.shape
    
    # Define the motion model
    warp_mode = cv2.MOTION_TRANSLATION
    
    # Define 2x3 or 3x3 matrices and initialize the matrix to identity
    if warp_mode == cv2.MOTION_HOMOGRAPHY :
        warp_matrix = np.eye(3, 3, dtype=np.float32)
    else :
        warp_matrix = np.eye(2, 3, dtype=np.float32)
    
    # Specify the number of iterations.
    number_of_iterations = 5000
    
    # Specify the threshold of the increment
    # in the correlation coefficient between two iterations
    termination_eps = 1e-10
    
    # Define termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)
    
    # Run the ECC algorithm. The results are stored in warp_matrix.
    (cc, warp_matrix) = cv2.findTransformECC (groundtruth,slammap_unaligned,warp_matrix, warp_mode, criteria)
    
    if warp_mode == cv2.MOTION_HOMOGRAPHY :
        # Use warpPerspective for Homography 
        slammap = cv2.warpPerspective (slammap_unaligned, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
    else :
        # Use warpAffine for Translation, Euclidean and Affine
        slammap = cv2.warpAffine(slammap_unaligned, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
    
    # Show the images
    cv2.imshow("Groundtruth", groundtruth)
    cv2.imshow("SLAM", slammap_unaligned)
    cv2.imshow("Aligned slam", slammap)
    #cv2.waitKey(0)
else:
    slammap = slammap_unaligned
#groundtruth =  cv2.imread(sys.argv[1],0)
#slammap =  cv2.imread(sys.argv[2],0)

# II. STEP 2, measure the error between the groundtruth amd the aligned slam map using knearest neighbour searvh, k = 1

options = {
    0: nearest_error,
    1: mse,
    2: issim
}
for attr, value in options.iteritems():
    value(groundtruth,slammap)
