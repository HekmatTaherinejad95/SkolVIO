#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 14 01:01:15 2020

@author: Hekmat Taherinejad
"""
import numpy as np
import cv2
import timeit 

def load_images(i):
    return cv2.imread('/home/hekmat/DATA/dataset/sequences/02/image_0/{0:06d}.png'.format(i), 0)

def ground_truth():
    file = '/home/hekmat/DATA/dataset/poses/02.txt'
    return np.genfromtxt(file, delimiter=' ',dtype=None)

def FeatureDetection():
    thresh = dict(threshold=25, nonmaxSuppression=True);
    fast = cv2.FastFeatureDetector_create(**thresh)
    return fast

def FeatureTracking(img_1, img_2, p1):

    lk_params = dict( winSize  = (21,21),
                      maxLevel = 3,
                      criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    p2, st, err = cv2.calcOpticalFlowPyrLK(img_1, img_2, p1, None, **lk_params)
    st = st.reshape(st.shape[0])
    
    good_new = p1[st==1]
    good_old = p2[st==1]

    return good_new,good_old

def intrinsic_K():
    return   np.array([[7.188560000000e+02, 0, 6.071928000000e+02],
              [0, 7.188560000000e+02, 1.852157000000e+02],
              [0, 0, 1]])
def getAbsoluteScale(f, i):
      x_pre, y_pre, z_pre = f[i-1][3], f[i-1][7], f[i-1][11]
      x    , y    , z     = f[i][3], f[i][7], f[i][11]
      scale = np.sqrt((x-x_pre)**2 + (y-y_pre)**2 + (z-z_pre)**2)
      return x, y, z, scale
GT = ground_truth()

#initialization
img_1 = load_images(0)
img_2 = load_images(1)

if len(img_1) == 3:
	gray_1 = cv2.cvtColor(img_1, cv2.COLOR_BGR2GRAY)
	gray_2 = cv2.cvtColor(img_2, cv2.COLOR_BGR2GRAY)
else:
	gray_1 = img_1
	gray_2 = img_2

#find the detector
detector = FeatureDetection()
kp1      = detector.detect(img_1)
p1       = np.array([ele.pt for ele in kp1],dtype='float32')
p1, p2   = FeatureTracking(gray_1, gray_2, p1)

#Camera parameters
fc = 718.8560
pp = (607.1928, 185.2157)
K  = intrinsic_K()

E, mask = cv2.findEssentialMat(p2, p1, fc, pp, cv2.RANSAC,0.999,1.0); 
_, R, t, mask = cv2.recoverPose(E, p2, p1,focal=fc, pp = pp);

#initialize some parameters
MAX_FRAME 	  = 2000
MIN_NUM_FEATURES  = 2000

preFeature = p2
preImage   = gray_2

R_f = R
t_f = t

start = timeit.default_timer()

traj = np.zeros((1000, 1000, 3), dtype=np.uint8)

maxError = 0

#play image sequences
for Frame_num in range(2, MAX_FRAME):

    print(Frame_num)

    if (len(preFeature) < MIN_NUM_FEATURES):
        feature   = detector.detect(preImage)
        preFeature = np.array([ele.pt for ele in feature],dtype='float32')

    #filename = imgs.format(numFrame)
    #print(filename)
    curImage_c = load_images(Frame_num)

    if len(curImage_c) == 3:
          curImage = cv2.cvtColor(curImage_c, cv2.COLOR_BGR2GRAY)
    else:
          curImage = curImage_c
    
    kp1 = detector.detect(curImage);
    preFeature, curFeature = FeatureTracking(preImage, curImage, preFeature)
    E, mask = cv2.findEssentialMat(curFeature, preFeature, fc, pp, cv2.RANSAC,0.999,1.0); 
    _, R, t, mask = cv2.recoverPose(E, curFeature, preFeature, focal=fc, pp = pp);

    truth_x, truth_y, truth_z, absolute_scale = getAbsoluteScale(GT, Frame_num)

    if absolute_scale > 0.1:  
        t_f = t_f + absolute_scale*R_f.dot(t)
        R_f = R.dot(R_f)

    preImage = curImage
    preFeature = curFeature
    

    ####Visualization of the result
    draw_x, draw_y = int(t_f[0]) + 200 , int(t_f[2]) + 100 ;
    draw_tx, draw_ty = int(truth_x) + 200 , int(truth_z) + 100

    curError = np.sqrt((t_f[0]-truth_x)**2 + (t_f[1]-truth_y)**2 + (t_f[2]-truth_z)**2)
    print('Current Error: ', curError)
    if (curError > maxError):
        maxError = curError

    cv2.circle(traj, (draw_x, draw_y) ,1, (0,0,255), 2);
    cv2.circle(traj, (draw_tx, draw_ty) ,1, (255,0,0), 2);

    cv2.rectangle(traj, (10, 30), (550, 50), (0,0,0), cv2.FILLED);
    text = "Coordinates: x ={0:02f}m y = {1:02f}m z = {2:02f}m".format(float(t_f[0]), float(t_f[1]), float(t_f[2]));
    cv2.putText(traj, text, (10,50), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8);

    cv2.drawKeypoints(curImage, kp1, curImage_c)
    cv2.imshow('image', curImage_c)
    cv2.imshow( "Trajectory", traj );
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
          break
  	#time.sleep(1)
# k = cv2.waitKey(0) & 0xFF
# if k == 27:
print('Maximum Error: ', maxError)
cv2.imwrite('map.png', traj);
stop = timeit.default_timer()
print(stop - start)
cv2.destroyAllWindows()

