import serial
import sys
import time
import signal
import matplotlib.pyplot as plt

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

while count < 500:
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

def to_int(byte):
    return int.from_bytes(j, 'little')

data2 = [[j for j in i] for i in data]

chan1 = []
chan2 = []

[chan1.entend(j[2:-2]) for j in data if j[1] == 0]
[chan2.entend(j[2:-2]) for j in data if j[1] == 1]

plt.figure(1)
plt.plot(chan1)

plt.figure(2)
plt.plot(chan2)

plt.show()


