import cv2
import numpy as np
import sys
import os
# load image
img = cv2.imread("/home/wangbard/sdp/demo2/4-way.png")

# create engine
engine = cv2.hfs.HfsSegment_create(img.shape[0], img.shape[1])
engine.setSlicSpixelSize(200)
# perform segmentation
# now "res" is a matrix of indices
# change the second parameter to "True" to get a rgb image for "res"
res = engine.performSegmentCpu(img, False)


# I find oftentimes black has the value 2
# manually changing black(2) to value 0
for i in range(len(res)):
    for j in range(len(res[0])):
            if res[i][j] == 2:
                res[i][j] = 0

# output file(txt)
np.savetxt("4-way.txt", res)

    
