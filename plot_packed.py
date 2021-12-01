import threading
import sys

import numpy as np
import matplotlib.pyplot as plt
import serial
import struct

ser = serial.Serial('COM4', 230400, timeout=10)

plt.figure(1)
plt.ion()
plt.show()

buf = []
buf2 = []
buf3 = []
ts = []
buf_lock = threading.Lock()

def update_func():
    garbage = ser.readline()
    print("sync")
    i = 0
    while True:
        read_bytes = ser.read(16)
        # [u, left, right] = struct.unpack('ff', read_bytes)
        data = struct.unpack('ffff', read_bytes)
        index = data[-1]
        if i != int(index):
            print("Data desync", i, data)
            return
        with buf_lock:
            if len(buf) == 1000:
                buf.pop(0)
                buf2.pop(0)
                buf3.pop(0)
                ts.pop(0)
            buf.append(data[0])
            buf2.append(data[1])
            buf3.append(data[2])
            ts.append(data[3])
        i += 1

read_thread = threading.Thread(group=None, daemon=True, target=update_func)
read_thread.start()

while True:
    plt.figure(1)
    plt.clf()
    plt.title("Raw and Filtered")
    with buf_lock:
        tmp = np.array(buf)
        tmp2 = np.array(buf2)
        tmp3 = np.array(buf3)
        _ts = np.array(ts)
    plt.plot(_ts, tmp, label="innovation")
    plt.plot(_ts, tmp2, label="encoder only")
    plt.plot(_ts, tmp3, label="fused")
    #plt.plot(tmp, tmp2, label="pos")
    plt.legend()
    plt.pause(0.25)
