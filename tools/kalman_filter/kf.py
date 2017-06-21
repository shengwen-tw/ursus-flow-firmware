import numpy as np
import matplotlib.pyplot as plt

csv = np.genfromtxt('input_data.csv', delimiter=',')

test_data_size = 100

#kalman filter final result
vx = [0 for k in range(0, test_data_size)]
vy = [0 for k in range(0, test_data_size)]

#kalman filter prior prediction
vx_predict = []
vy_predict = []

#prediction control variable u (from accelerometer)
accel_ax = csv[:, 2]
accel_ay = csv[:, 3]
delta_t = csv[:, 4]
time = csv[:, 5]

#measurement state variables z (from optical flow)
flow_vx = csv[:, 0]
flow_vy = csv[:, 1]

#process error covariance matrix
p11 = 0; p12 = 0
p21 = 0; p22 = 0

#measurement error covariance matrix
q11 = 0; q12 = 0
q21 = 0; q22 = 0

#kalman gain matrix
g11 = 0; g12 = 0
g21 = 0; g22 = 0

vx[0] = 0
vy[0] = 0

def kalman_filter():
    for k in range(1, test_data_size):
        #prediction
        vx_predict = vx[k - 1] + accel_ax[k] * delta_t
        vy_predict = vy[k - 1] + accel_ay[k] * delta_t

        p11 = p11 + q11; p12 = 0
        p21 = 0        ; p22 = p22 + q22

        #update
        g11 = (p11) / (p11 + r11); g12 = 0
        g21 = 0                  ; g22 = (p22) / (p22 + r22)

        vx = vx_predict[k] + g11(flow_vx[k] - vx_predict[k])
        vy = vy_predict[k] + g22(flow_vy[k] - vy_predict[k])

        p11 = (1 - g11) * p11; p12 = 0
        p21 = 0              ; p22 = (1 - g22) * p22

#plot result
plt.figure('Kalman Filter - x')
plt.plot(time, flow_vx)
plt.plot(time, accel_ax)
plt.legend(['flow_vx', 'accel_ax'], loc='upper left')

plt.figure('Kalman Filter - y')
plt.plot(time, flow_vy)
plt.plot(time, accel_ay)
plt.legend(['flow_vy', 'accel_ay'], loc='upper left')

plt.show()
