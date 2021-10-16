import threading
import sys

import numpy as np
import matplotlib.pyplot as plt
import serial

ser = serial.Serial('COM4', 115200, timeout=1)

plt.figure(1)
plt.ion()
plt.show()

buf = []
buf2 = []
buf_lock = threading.Lock()

def update_func():
    while True:
        in_line = ser.readline()
        try:
            s = in_line.decode('utf-8').split(" ")
            filt = float(s[0])
            raw = float(s[1])
            with buf_lock:
                if len(buf) == 1000:
                    buf.pop(0)
                    buf2.pop(0)
                buf.append(filt)
                buf2.append(raw)
        except:
            pass

read_thread = threading.Thread(group=None, daemon=True, target=update_func)
read_thread.start()

while True:
    plt.figure(1)
    plt.clf()
    plt.title("Raw and Filtered")
    with buf_lock:
        tmp = np.array(buf)
        tmp2 = np.array(buf2)
    plt.plot(range(len(tmp)), tmp, label="filt")
    plt.plot(range(len(tmp2)), tmp2, label="raw")
    plt.legend()
    plt.pause(0.25)
