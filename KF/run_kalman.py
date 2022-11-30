import kalman
import arduino_udp
import numpy as np
import datetime

# For logging
def array_to_csv(array):
    s = ""
    for elem in array:
        s += str(elem) + ","
    return s[:-1]

timestep = 0.032    # Timestep
sigma_j = 0.10      # Jerk std. dev.
bias_z = -9.125025  # Static value of accelerometer
offset_z = -0.008   # Offset from sensor to bottom of basket [m]

dt = timestep / 2
A = np.array([  [1, dt,  0.5*dt**2],
                [0,  1,         dt],
                [0,  0,          1] ])
B  = np.array([0])
Hd = np.array([ [1, 0, 0] ])
Ha = np.array([ [0, 0, 1] ])
Q  = np.array([ [1/36*dt**6, 1/12*dt**5, 1/6*dt**4],
                [1/12*dt**5,  1/6*dt**4, 1/2*dt**3],
                [1/6*dt**4,   1/2*dt**3,     dt**2] ]) * sigma_j**2
Rd  = [0.003 **2]       # Range sensor variance
Ra  = [0.120 **2]       # Accelerometer variance

filter = kalman.KalmanFilter(A, B, Hd, Q, Rd)
arduino = arduino_udp.Arduino('192.168.10.240', 8888)

# Store times
tnow = datetime.datetime.now()
tstart = tnow
tlast = tnow
datetimestr = tnow.strftime("%Y%m%dT%H%M%S")
fname = "logs_kalman/" + datetimestr + ".csv"
print("Starting logging to", fname)
samples = 0
with open(fname, "x") as f:
    # CSV header
    f.write("accel_z,range_sensor,d_est,v_est,a_est,t\n")
    # Initialize 
    est = [0.200, 0, 0]
    filter.x_est = np.array([est]).T
    # Read at fixed interval
    try:
        while True:
            
            tnow = datetime.datetime.now()
            delta_t = (tnow - tlast).total_seconds()
            while delta_t < timestep: # Loop until next update is scheduled - attempt to keep samplerate capped
                tnow = datetime.datetime.now()
                delta_t = (tnow - tlast).total_seconds()

            # Get current measurement
            data = arduino.send_receive(float(Hd@est))
            data[2] = (data[2]) * 9.81 # Convert to m/s^2
            data[2] = data[2] - bias_z # Remove bias
            data[2] = -data[2]         # Change direction of z-axis
            data[3] = data[3] * 1e-3 + offset_z # Convert mm to m

            # Sensor fusion
            filter.H = Ha; filter.R = Ra
            est = filter.update(data[2], [0]) # Update with accelerometer reading
            filter.H = Hd; filter.R = Rd
            est = filter.update(data[3], [0]) # Update with range reading
            # Log
            delta_t  = tnow - tstart
            t = delta_t.total_seconds()
            log_data = np.array([data[2], data[3], est[0, 0], est[1, 0], est[2, 0], t])
            log_str = array_to_csv(log_data)
            f.write(log_str + "\n")
            samples += 1
            tlast = tnow
            print(Hd@est)

    except KeyboardInterrupt:
            print('\nProgram terminated with keyboard interrupt.')
        
    finally:
        delta_t = tlast - tstart
        print("Finished in", delta_t.total_seconds(), "s")
        print("Average samplerate:", samples/delta_t.total_seconds(), "Hz")
