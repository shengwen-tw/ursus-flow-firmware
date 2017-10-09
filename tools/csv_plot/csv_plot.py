import numpy as np
import matplotlib.pyplot as plt

csv = np.genfromtxt('accel_log.csv', delimiter=',')
data_size = len(csv[:, 0])

#prediction control variable u (from accelerometer)
time = csv[:, 0]
end_time = time[data_size - 1]
data1 = csv[:, 1]
data2 = csv[:, 2]

#plot result
plt.figure('csv plot tool')
plt.xlim(0, end_time)
plt.plot(time, data1)
plt.plot(time, data2)
plt.xlabel('time (s)', fontsize=18)
plt.ylabel('acceleration (a/s)', fontsize=18)
plt.legend(['low-pass filter', 'raw data'], loc='upper left')

plt.show()
