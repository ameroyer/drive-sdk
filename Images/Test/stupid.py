'''
Simple and fast image transforms to mimic:
 - brightness
 - contrast
 - erosion 
 - dilation
'''

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


def get_cnt_bounding_box(cnt):
    print cnt.shape
    print cnt
    maxx = 0
    minx = 1920
    maxy = 0
    miny = 720
    for z in cnt:
        x, y = z[0]
        if x < minx:
            minx = x
        elif x > maxx:
            maxx = x
        if y < miny:
            miny = y
        elif y > maxy:
            maxy = y
    return (minx, miny, maxx, maxy)

# 1 = outside, 0 = inside
def get_topology(cnt, cnts):
    (t1, l1, b1, r1) = cnt
    for t2, l2, b2, r2 in cnts:
        if t1 > t2 and l1 > l2 and b1 < b2 and r1 < r2:
            return 0
    return 1
        
        

# Image data
image = cv2.imread('../default_background.ppm') 
mask = np.zeros_like(image)

kernel = np.ones((10,10),np.uint8)
image = cv2.morphologyEx(image, cv2.MORPH_GRADIENT,  kernel)
image = cv2.dilate(image,kernel,iterations = 3)
image = 255 - image
image = cv2.cvtColor( image, cv2.COLOR_BGR2GRAY );
_, image = cv2.threshold(image,200,255,0)

plt.imshow(image,  cmap = cm.Greys_r)
plt.show()

# Find contours
_, contours, hier = cv2.findContours(image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
tracks = {}
ntracks = 0
for cnt in contours:
    if 20000<cv2.contourArea(cnt):
        tracks[ntracks] = [cnt, (get_cnt_bounding_box(cnt))]
        ntracks +=1

bbs = [x[1] for x in tracks.itervalues()]
for k, v in tracks.iteritems():
    tracks[k].append(get_topology(v[1], bbs))

# Fill mask
cv2.drawContours(mask,[x[0] for x in tracks.itervalues() if x[2]], 0, (255,255,255), -1)
cv2.drawContours(mask,[x[0] for x in tracks.itervalues() if not x[2]], 0, (0, 0, 0), -1)

plt.imshow(mask)
plt.show()
