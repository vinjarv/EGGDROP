import arduino_udp
import datetime
import numpy as np
import random
from time import sleep

def array_to_csv(array):
    s = ""
    for elem in array:
        s += str(elem) + ","
    return s[:-1]

arduino = arduino_udp.Arduino('192.168.10.240', 8888)

dt = datetime.datetime.now()
datetimestr = dt.strftime("%Y%m%dT%H%M%S")
fname = "logs_data/" + datetimestr + ".csv"
print("Starting logging to", fname)
with open(fname, "x") as f:
    # CSV header
    f.write("accel_x,accel_y,accel_z,range_sensor,t\n")
    t0 = datetime.datetime.now()
    for i in range(500):
        dt = datetime.datetime.now() - t0
        t = dt.total_seconds()
        
        array = arduino.send_receive(0)

        # array = np.array([random.gauss(0, 0.3), random.gauss(0, 0.5), random.gauss(1, 0.3), random.gauss(100, 0.1)])
        # sleep(0.001)

        array = np.append(array, t)
        f.write(array_to_csv(array) + "\n")        
