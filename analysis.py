#!/usr/bin/python
import sys
import matplotlib.pyplot as plt
import numpy as np

fd = open(sys.argv[1], 'rb')
data = [fd.read(36) for i in range(1080000//36)]
fd.close()

chan1 = []
chan2 = []

[chan1.extend(j[2:-2]) for j in data if j[1] == 0]
[chan2.extend(j[2:-2]) for j in data if j[1] == 1]

chan1_array = np.array(chan1)
chan2_array = np.array(chan2)

chan1_fft = np.fft.fft(chan1_array[0:80000]/255)/80000
chan2_fft = np.fft.fft(chan2_array[0:80000]/255)/80000

plt.figure(1)
plt.plot(chan1)

plt.figure(2)
plt.plot(chan2)

plt.figure(3)
plt.plot(chan1_fft[1:40000])

plt.figure(4)
plt.plot(chan2_fft[1:40000])

plt.show()

