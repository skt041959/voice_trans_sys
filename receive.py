#!/usr/bin/python
import serial
import sys
import time
import wave
import matplotlib.pyplot as plt
import scipy.signal as signal
import numpy as np

usb_ids = [
        '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_5CF2855D3335-if00',
        '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_5CF385593335-if00',
        '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_5CF3644E3335-if00'
]

data = []
count = 0

port = None

def sig_handler(signum, frame):
    if port:
        port.close()

signal.signal(signal.SIGINT,sig_handler)
signal.signal(signal.SIGTERM,sig_handler)

while not port:
    try:
        port = serial.Serial(usb_ids[int(sys.argv[1])])
        print(usb_ids[int(sys.argv[1])])
        print("port {0} opened".format(sys.argv[1]))
    except serial.SerialException:
        print(usb_ids[int(sys.argv[1])])
        print("port {0} not plugin".format(sys.argv[1]))
        time.sleep(3)

while count < 500*60:
    src = port.read(36)
    data.append(src)
    print(src)
    count += 1

if port:
    port.close();

print("end receive")

fd = open("{0}".format(time.strftime('%m-%d_%H_%M_%S')), 'wb')

[fd.write(e) for e in data]

fd.close()

chan1 = []
chan2 = []

[chan1.extend(j[2:-2]) for j in data if j[1] == 0]
[chan2.extend(j[2:-2]) for j in data if j[1] == 1]

f1 = wave.open('{0}_chan1.wav'.format(time.strftime('%m-%d_%H_%M_%S')),'wb')
f1.setnchannels(1)
f1.setsampwidth(1)
f1.setframerate(8000)
f1.writeframes(np.array(chan1).tostring())
f1.close()

f2 = wave.open('{0}_chan2.wav'.format(time.strftime('%m-%d_%H_%M_%S')),'wb')
f2.setnchannels(1)
f2.setsampwidth(1)
f2.setframerate(8000)
f2.writeframes(np.array(chan2).tostring())
f2.close()

chan1_array = np.array()
chan2_array = np.array()

chan1_fft = np.fft.fft(chan1_array[0:80000]/255)/80000
chan2_fft = np.fft.fft(chan2_array[0:80000]/255)/80000

plt.figure(1)
plt.plot(chan1)

plt.figure(2)
plt.plot(chan2)

plt.figure(3)
plt.plot(chan1_fft)

plt.figure(4)
plt.plot(chan2_fft)

plt.show()

