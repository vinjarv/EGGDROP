import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("TKAgg")

plt.axis([0, 100, 0, 1])

for i in range(100):
    y = np.random.random()
    plt.scatter(i, y)
    plt.pause(1e-9)

plt.show()
# Plotting live is pretty slow
