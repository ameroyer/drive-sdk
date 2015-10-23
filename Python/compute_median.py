from numpy import median, array
from PIL import Image
import sys, os

# Compute median image from sequence of images given as input
if __name__ == '__main__':
    imgs = []
    for img in sys.argv[2:]:
        if not os.path.isfile(img):
            break
        imgs.append(array(Image.open(img)))

    aux = median(array(imgs), axis=0)
    aux = aux.astype('uint8')
    im = Image.fromarray(aux)
    im.save(sys.argv[1])
