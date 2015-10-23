from numpy import median, array, abs
import matplotlib.pyplot as plt
from PIL import Image
import sys, os

# Compute median image from sequence of images given as input
if __name__ == '__main__':
    im1 = array(Image.open(sys.argv[1]))
    im2 = array(Image.open(sys.argv[2]))
    im3 = abs(im1 - im2)
    im = Image.fromarray(im3)
    im.show()
