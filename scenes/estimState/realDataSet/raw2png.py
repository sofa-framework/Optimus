#!/usr/bin/env python3
import cv2
import numpy as np
import sys
w=640
h=480

if __name__ == '__main__':
    with open(sys.argv[1],'rb') as f:
        for i,ima in enumerate(iter(lambda: f.read(w*h), b'')):
            ima=np.frombuffer(ima,dtype=np.uint8).reshape((h,w))
            #outputStr = f'{sys.argv[2]}_{i:06}.png'
            outputStr = "{0}_{1:06d}.png".format(sys.argv[2], i )
            cv2.imwrite(outputStr,ima)

