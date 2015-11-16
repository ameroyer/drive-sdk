"""
Get view of the track and discretize it to define states for the Machine Learning step
"""
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from random import random, randint

def dist(x1, y1, x2, y2):
    """
    Return distance between (x1, y1) and (x2, y2)
    """
    return np.sqrt((y2 - y1)**2 + (x2 - x1)**2)


def get_normal(x1, y1, x2, y2):
    """
    Return a (non-normalized) orthogonal vector to the line (x1, y1)-(x2, y2)
    """
    a = (y2 - y1) / (x2 - x1)
    b = y1 - a * x1
    return (a, b)


def project(x, y, xin, yin, dx, dy):
    """
    Return the projection of (x, y) on the line passing through (xin, yin) with direction vector (dx, dy)
    """
    norm = dist(0, 0, dx, dy)
    scal = (dx * (x - xin) + dy * (y - yin)) / norm**2
    return xin + scal * dx, yin + scal * dy


def scalar(x1, y1, x2, y2):
    """
    Return scalar product between vectors (x1, y1) and (x2, y2)
    """
    return x1 * x2 + y1 * y2

def get_cosine(x1, y1, x2, y2):
    return scalar(x1, y1, x2, y2) / (dist(0, 0, x1, y1) * dist(0, 0, x2, y2))


def get_curvature(x1, y1, x2, y2):
    c1 = get_cosine(abs(x2 - x1), abs(y2 - y1), 0, 1);
    c2 = get_cosine(abs(x2 - x1), abs(y2 - y1), 1, 0);
    return min(c1, c2);


def get_cnt_bounding_box(cnt, width=1920, height=720):
    """
    Give a contour (list of points), return its bounding box
    """
    maxx, maxy = 0, 0
    minx, miny = width, height
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
# TODO return true false instead
def get_topology(cnt, cnts):
    """
    Return 1 iff the contour cnt is inside another contour
    """
    (t1, l1, b1, r1) = cnt
    for t2, l2, b2, r2 in cnts:
        if t1 > t2 and l1 > l2 and b1 < b2 and r1 < r2:
            return 0
    return 1


def get_isobarycenter(pts):
    area = sum([(p[0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * p[1]) for i, p in enumerate(pts)]) / 2.
    xg = np.sum([ (p[0] + pts[(i + 1) % len(pts)][0]) * (p[0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * p[1]) for i, p in enumerate(pts)]) / (6 * area)
    yg = np.sum([ (p[1] + pts[(i + 1) % len(pts)][1]) * (p[0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * p[1]) for i, p in enumerate(pts)]) / (6 * area)
    return xg, yg
        
        
if __name__ == "__main__":

    ##### 1. Get the track mask

    # Load Image
    image = cv2.imread('/home/cvml1/Code/Images/default_background.ppm') 
    mask = np.zeros_like(image)

    print "Extract track mask"
    # Apply dilation and contour detection
    kernel = np.ones((10,10),np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_GRADIENT,  kernel)
    image = cv2.dilate(image,kernel,iterations = 3)
    # Apply black and white thresholding
    image = 255 - image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, image = cv2.threshold(image,200,255,0)

    # Apply horizontal symmetry to remove light noise
    middle = image.shape[1] / 2
    image[:, middle:] = np.fliplr(image)[:, middle:]


    # Find largest contours of white zomes (ie, outlines of the track)
    minarea = 20000
    _, contours, hier = cv2.findContours(image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    tracks = {}
    ntracks = 0
    for cnt in contours:
        if minarea < cv2.contourArea(cnt):
            tracks[ntracks] = [cnt, (get_cnt_bounding_box(cnt))]
            ntracks +=1

    # Find bounding box and inside/outside relative positions
    bbs = [x[1] for x in tracks.itervalues()]
    for k, v in tracks.iteritems():
        tracks[k].append(get_topology(v[1], bbs))

    # Draw final track's mask
    cv2.drawContours(mask,[x[0] for x in tracks.itervalues() if x[2]], 0, (255,255,255), -1)
    cv2.drawContours(mask,[x[0] for x in tracks.itervalues() if not x[2]], 0, (0, 0, 0), -1)


    ####### Discretize the track (Note: Finetuned for the first track)
    print "Discretize track"
    dh = 50 # Distance between vertical segments
    dv = 4  # Number of horizontal segments
    if len(sys.argv) > 1:
        dh = int(sys.argv[1])
    if len(sys.argv) > 2:
        dv = int(sys.argv[2])
    
    inside = [x[0] for x in tracks.itervalues() if not x[2]][0]
    outside = [x[0] for x in tracks.itervalues() if x[2]][0]
    
    vsegments = []
    indx_out, indx_in = 0, 0
    x_out, y_out = outside[indx_out][0] # Current point on the outside track
    x_in, y_in = 0, 0 # Current point on the inside track

    while 1:
        # Break
        if indx_out >= len(outside):
            break

        # Find closest "before" and "after" points on the inside track
        bf_fit = 10000
        af_fit = 10000
        step = 5
        x1_out, y1_out = outside[(indx_out - step) % len(outside)][0]
        x2_out, y2_out = outside[(indx_out + step) % len(outside)][0]
        x1_in, y1_in, x2_in, y2_in = 0, 0, 0, 0
        for i in xrange(0, len(inside)):
            p, q = inside[i][0]
            d = dist(x_out, y_out, p, q)
            # Before
            if scalar(p - x_out, q - y_out, x1_out - x_out,  y1_out - y_out) > 0:
                if d < bf_fit:
                    bf_fit = d
                    x1_in, y1_in = p, q
            # After
            elif scalar(p - x_out, q - y_out, x2_out - x_out, y2_out - y_out) > 0:
                if d < af_fit:
                    af_fit = d
                    x2_in, y2_in = p, q

        # Project
        x_in , y_in = project(x_out, y_out, x1_in, y1_in, x2_in - x1_in, y2_in - y1_in)
        vsegments.append((x_out, y_out, x_in, y_in))

        # Plot
        c = [int(random() * 255) for _ in xrange(3)]
        cv2.circle(mask,(int(x_out),int(y_out)), 3, c,-11)
        cv2.circle(mask,(int(x_in),int(y_in)), 3, c, -2)
        cv2.line(mask, (int(x_out), int(y_out)), (int(x_in), int(y_in)), c, 2)

        # Find next point on outside track
        distance = 0
        while 1:
            indx_out += 1
            if indx_out >= len(outside):
                break
            x, y = outside[indx_out][0] 
            diff = dist(x, y, x_out, y_out)
            # If above delta, find best point
            if distance + diff > dh:
                x_out +=  (x - x_out) * (dh - distance) / diff
                y_out += (y - y_out) * (dh - distance) / diff
                break
            else:
                distance += diff
                x_out, y_out = x, y


    # Find the horizontal segments and compute the parts centroids
    centroids = []
    previous = [0] * (dv + 1)
    first = [0] * (dv + 1)
    startline = 0
    startline_pixel = (855, 164)
    closest = 100000
    vsegments.append(vsegments[0])
    for i, (x1, y1, xin1, yin1) in enumerate(vsegments):
        d = dist(x1, y1, xin1, yin1)
        x2, y2, xin2, yin2 = vsegments[(i - 1) % len(vsegments)]
        top = [(x1, y1), (x2, y2)]
        for j in xrange(dv + 1):
            # Compute intersection on the vertical segment
            p, q = x1 + j * d /dv * (xin1 - x1) / d, y1 + j * d / dv * (yin1 - y1) / d
            if j == dv:
                p, q = xin1, yin1
            bottom = [previous[j], (p, q)]            
            cv2.circle(mask,(int(p),int(q)), 3, (255, 0, 0), -2)
            if i == 0:
                first[j] = (p, q)
            else:
                cv2.line(mask, (int(p), int(q)), (int(previous[j][0]), int(previous[j][1])), (255, 0, 0), 2)
                if j != 0 and i != 0:
                    xg, yg = get_isobarycenter(top + bottom)
                    centroids.append([(xg, yg), i - 1, j - 1, get_curvature(x1, y1, xin1, yin1), 0]) #centroid, vertical id, horizontal id, curvature, startline
                    # Compute distane to start line 
                    d = dist(xg, yg, startline_pixel[0], startline_pixel[1])
                    if d < closest:
                        closest = d
                        startline = i - 1
                    cv2.circle(mask,(int(xg),int(yg)), 3, (0, 255, 0) if i != 36 else  (0, 0, 255), -2)
            top = [bottom[1], bottom[0]]
            previous[j] = (p, q)

    # Set startline
    for i, x in enumerate(centroids):
        if x[1] == startline:
            x[-1] = 1
	    centroids[i+1][-1] = 1
	    centroids[i+2][-1] = 1
	    break
    centroids = centroids[i:] + centroids[:i]

    # Save
    with open("centroids_h%d_v%d.txt" %(dh, dv), "w") as f:
        f.write("\n".join("%f %f %d %d %f %d" %(ctr[0], ctr[1], i, j, curv, start) for ctr, i, j, curv, start in centroids))
    import Image
    Image.fromarray(mask).save("discretized_track_h%d_v%d.png" %(dh, dv))
