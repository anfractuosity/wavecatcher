#!/usr/bin/python

from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.signal import kaiserord, lfilter, firwin, freqz
from itertools import tee

# Sampling rate 5.5Msps, this is higher than the 4.5Msps I believe SPI was set to
# need to check why this is the case

sf = 5500000

t = []
flip = False

with open('dump.bin', 'rb') as input_file:

    data = input_file.read()
    data2 = data
   
    """
    for block in range(0,len(data)//64) :
        for f in range(63,-1,-1):
            new = (block*64)+f
            data2.append(data[new])
    """

    n = 1
    for b in data2:
    
        sumv = 0.0
        for i in range(0,8):
            if flip:
                bit = b & 128
            
                if bit == 128:
                    t.append(n)
                else:
                    t.append(0)
        
                b <<= 1
            else:

                bit = b & 1

                if bit == 1:
                    t.append(n)
                else:
                    t.append(0)
                b >>= 1
    
def window(iterable, size):
    iters = tee(iterable, size)
    for i in range(1, size):
        for each in iters[i:]:
            next(each, None)
    return zip(*iters)

# Efficient solution from - https://stackoverflow.com/questions/13728392/moving-average-or-running-mean
def running_mean(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

wavfile.write("test.wav", sf, (np.array(running_mean(t,200)*1.5)-0.5))
