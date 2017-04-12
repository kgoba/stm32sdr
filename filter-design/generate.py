#!/usr/bin/env python

import scipy.signal as spsig
import numpy as np
import sys

def generateHilbert(N):
    middle = N / 2
    result = np.zeros(N)
    for i in range(N):
        j = i - middle
        if (j % 2) == 0:
            continue
        result[i] = 2 / (np.pi * j)
    return result
    
    
def writeArray(title, stream, h):
    stream.write("%s\n" % title)
    idx = 0
    for x in h:
        stream.write("%ff" % x)
        stream.write(", ")
        idx += 1
        if idx % 8 == 0:
            stream.write("\n")
        
    if idx % 8 != 0:
      stream.write("\n")


h = generateHilbert(127)
writeArray("Hilbert 127", sys.stdout, h)

h = spsig.firwin(127, 0.95 / 2)
writeArray("Lowpass 1/2", sys.stdout, h)

h = spsig.firwin(127, 0.95 / 3)
writeArray("Lowpass 1/3", sys.stdout, h)
