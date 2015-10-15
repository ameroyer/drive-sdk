#!/usr/bin/python
import sysv_ipc
import numpy as np
import pylab

# attach to shared memory with key 192012003
S=sysv_ipc.SharedMemory(key=192012003)

# do stuff with the shared mem
for i in range(10):
	# read contents into numpy array
	x=np.frombuffer(S.read(1696*720*3),dtype=np.uint8).reshape(720,1696,3)
	print x.mean()

# finally, detach from memory again
S.detach()

#test save image
import Image
im = Image.fromarray(x)
im.save('test_outfile.jpg')
