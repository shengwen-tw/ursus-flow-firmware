import numpy as np
import matplotlib.pyplot as plt

csv = np.genfromtxt('flow.csv', delimiter=',')
data_size = len(csv[:, 0])

#prediction control variable u (from accelerometer)
data1 = csv[:, 0]
data2 = csv[:, 1]

#plot result
plt.figure('Optical Flow')
plt.xlim(0, data_size)
plt.plot(data1)
plt.plot(data2)
plt.xlabel('time (t)', fontsize=18)
plt.legend(['kalman filter flow (m/s)', 'raw flow vx (m/s)'], loc='upper left')

plt.show()
