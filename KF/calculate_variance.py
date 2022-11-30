import pandas as pd
import numpy as np
import sys

filename = sys.argv[1]
df = pd.read_csv(filename)
print("File read")
print(df.info())

accel = df[["accel_x", "accel_y", "accel_z", "range_sensor"]]
accel = accel.assign(accel_x = df.accel_x * 9.81)
accel = accel.assign(accel_y = df.accel_y * 9.81)
accel = accel.assign(accel_z = df.accel_z * 9.81)
accel = accel.assign(range_sensor = df.range_sensor * 1e-3)

print("\nCalculating covariance matrix")
print(accel.cov())

print("\nMean values")
print(accel.mean())

print("\nStd.dev accel_z:", np.std(accel["accel_z"].to_numpy()))
print("Std.dev range_sensor:", np.std(accel["range_sensor"].to_numpy()))

delta_t = np.convolve(df["t"].to_numpy(), [1, -1])[1:-1]
print("\nMean timestep:", round(np.mean(delta_t)*1e3, 3), "ms")
print("Timestep std. dev.", round(np.std(delta_t)*1e3, 3), "ms")
