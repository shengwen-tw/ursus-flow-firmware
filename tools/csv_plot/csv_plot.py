import numpy as np
import matplotlib.pyplot as plt

csv = np.genfromtxt('flow.csv', delimiter=',')
data_size = len(csv[:, 0])

#prediction control variable u (from accelerometer)
time = csv[:, 0]
end_time = time[data_size - 1] - 3.2
data1 = csv[:, 1] / 100
data2 = csv[:, 2] / 100

#plot result
plt.figure('csv plot tool')
plt.xlim(0, end_time)
plt.plot(time, data1)
plt.plot(time, data2)
plt.xlabel('time (s)', fontsize=18)
plt.ylabel('velocity (m/s)', fontsize=18)
plt.legend(['kalman filter', 'optical flow'], loc='upper left')

plt.show()
