import matplotlib.pyplot as plt
import pandas as pd
import sys
import matplotlib
matplotlib.use("TKAgg")
filename = sys.argv[1]
df = pd.read_csv(filename)
print("File read")
print(df.info())

plt.figure()
plt.plot(df["t"], df["range_sensor"], label="Range sensor")
plt.plot(df["t"], df["d_est"], label="$\hat x_d$")
plt.legend()
plt.grid(True)
plt.xlabel("Time (s)")
plt.xlim(0, df["t"].max())
plt.ylabel("Distance (m)")
plt.title("Distance")

plt.figure()
plt.plot(df["t"], df["accel_z"], label="Accelerometer Z-axis")
plt.plot(df["t"], df["a_est"], label="$\hat x_a$")
plt.legend()
plt.grid(True)
plt.xlabel("Time (s)")
plt.xlim(0, df["t"].max())
plt.ylabel("Acceleration ($m/s^2$)")
plt.title("Acceleration")

plt.show()
