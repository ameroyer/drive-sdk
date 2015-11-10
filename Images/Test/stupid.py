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


def dist(x1, y1, x2, y2):
    return np.sqrt((y2 - y1)**2 + (x2 - x1)**2)

def get_normal(x1, y1, x2, y2):
    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return (a, b)


def project(x, y, xin, yin, dx, dy):
    norm = dist(0, 0, dx, dy)
    scal = (dx * (x - xin) + dy * (y - yin)) / norm**2
    return xin + scal * dx, yin + scal * dy

def scalar(x1, y1, x2, y2):
    return x1 * x2 + y1 * y2

def get_cnt_bounding_box(cnt):
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
#TODO inside of X and outside of X
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


# Add horizontal  symmetry
middle = image.shape[1] / 2
#print middle
image[:, middle:] = np.fliplr(image)[:, middle:]


#plt.imshow(image)
#plt.show()
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


# Finetuned for 2 trcaks
# Find vertical partitions
delta = 50 # length in pixels
from random import randint
segments = []
inside = [x[0] for x in tracks.itervalues() if not x[2]][0]
outside = [x[0] for x in tracks.itervalues() if x[2]][0]
indx_out = randint(0, 500)
x, y = outside[0][0]
xin, yin = 0, 0
segments = []
indx_in = 0
#indx_out = 0
#print indx_out
indx_out = 60
indx_out = 0
x, y = outside[indx_out][0]
while 1:
    if indx_out >= len(outside):
        break
    from random import random
    c = [int(random() * 255) for _ in xrange(3)]
    cv2.circle(mask,(int(x),int(y)), 3, c,-11)

    # Project point
    # Assume inside/outside parallel

    # getcouldn't connect to display "localhost: closeset point
    fit = 100000
    bf_fit = 10000
    af_fit = 10000
    x1_out, y1_out = outside[(indx_out - 5) % len(outside)][0]
    x2_out, y2_out = outside[(indx_out + 5) % len(outside)][0]
    #print x2_out, y2_out, x1_out, y1_out
    x1_in, y1_in, x2_in, y2_in = 0, 0, 0, 0
    #cv2.circle(mask,(int(x1_out),int(y1_out)), 10, c, -2)
    #cv2.circle(mask,(int(x2_out),int(y2_out)), 10, c, -2)
    #print x2_out - x, y2_out - y, x1_out - x, y1_out - y
    for i in xrange(0, len(inside)):
        p, q = inside[i][0]
        #print p, q
        d = dist(x, y, p, q)
        # Before
        a= scalar(p - x, q - y, x2_out - x, y2_out - y)
        if scalar(p - x, q - y, x1_out - x,  y1_out - y) > 0:
            #print "Case 1"
            if d < bf_fit:
                bf_fit = d
                x1_in, y1_in = p, q
        elif scalar(p - x, q - y, x2_out - x, y2_out - y) > 0:
            #print "fuuuuuuuuuuuuuuuu"
            #print "yo", x2_out, y2_out
            if d < af_fit:
                af_fit = d
                x2_in, y2_in = p, q
        else:
            pass
            #print a <= 0
            #print "What te fuck"
    # Projection
    #print x1_in, y1_in, x2_in, y2_in
    xin ,yin = project(x, y, x1_in, y1_in, x2_in - x1_in, y2_in - y1_in)
    #xin, yin = x2_in, y2_in
    #print xin, yin
    #cv2.circle(mask,(int(x1_in),int(y1_in)), 10, (0, 0, 255), -2)
    #cv2.circle(mask,(int(x2_in),int(y2_in)), 10, (255, 0, 0), -2)
    cv2.circle(mask,(int(xin),int(yin)), 3, c, -2)
    cv2.line(mask, (int(x), int(y)), (int(xin), int(yin)), c, 2)
    #break
    segments.append((x, y, xin, yin))
    
    
            
        
        
    #print inside[indx_in][0]
    # Find projection on inside lane Hopefully points are given in order
    #print inside[indx_in + 1][0]
    #break


    # Find next point
    distance = 0
    while 1:
        indx_out += 1
        if indx_out >= len(outside):
            break
        x1, y1 = outside[indx_out][0] 
        diff = dist(x1, y1, x, y)
        # If above delta, find best point
        if distance + diff > delta:
            x +=  (x1 - x) * (delta - distance) / diff
            y += (y1 - y) * (delta - distance) / diff
            break
        else:
            distance += diff
            x, y = x1, y1
        
# vertical splits
div = 3
old = [0] * 4
first = [0] * 4
t = False
for i, (x1, y1, xin1, yin1) in enumerate(segments):
    d = dist(x1, y1, xin1, yin1)
    aux = []
    for j in xrange(div+1):
        p, q = x1 + j * d /3 * (xin1 - x1) / d, y1 + j * d /3 * (yin1 - y1) / d
        cv2.circle(mask,(int(p),int(q)), 3, (255, 0, 0), -2)
        if t:
            cv2.line(mask, (int(p), int(q)), (int(old[j][0]), int(old[j][1])), (255, 0, 0), 2)    
        else:
            first[j] = (p, q)
        old[j] = (p, q)
    t = True
        #cv2.circle(mask,(int(p2),int(q2)), 3, (255, 0, 0), -2)
# Fill last
print first, old
for j in xrange(div):
    cv2.line(mask, (int(old[j][0]), int(old[j][1])), (int(first[j][0]), int(first[j][1])), (255, 0, 0), 2)
    
plt.imshow(mask)
import Image
#Image.fromarray(mask).save("stupid.png")
plt.show()
