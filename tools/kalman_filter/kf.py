import numpy as np
import matplotlib.pyplot as plt

csv = np.genfromtxt('input_data.csv', delimiter=',')

test_data_size = len(csv[:, 0])

#kalman filter final result
vx = [0 for k in range(0, test_data_size)]
vy = [0 for k in range(0, test_data_size)]

#kalman filter prior prediction
vx_predict = [0 for k in range(0, test_data_size)]
vy_predict = [0 for k in range(0, test_data_size)]

#prediction control variable u (from accelerometer)
accel_ax = csv[:, 2]
accel_ay = csv[:, 3]
delta_t = csv[:, 4]
time = csv[:, 5]

#measurement state variables z (from optical flow)
flow_vx = csv[:, 0]
flow_vy = csv[:, 1]

#process error covariance matrix
p11 = [0 for k in range(0, test_data_size)]; p12 = [0 for k in range(0, test_data_size)]
p21 = [0 for k in range(0, test_data_size)]; p22 = [0 for k in range(0, test_data_size)]

#measurement error covariance matrix
q11 = 0.75; q12 = 0
q21 = 0   ; q22 = 0.75

r11 = 70.0; r12 = 0
r21 = 0   ; r22 = 70.0

#kalman gain matrix
g11 = 0; g12 = 0
g21 = 0; g22 = 0

vx[0] = 0
vy[0] = 0

def kalman_filter():
    for k in range(1, test_data_size):
        #prediction
        vx_predict[k] = vx[k - 1] + (accel_ax[k] * delta_t[k])
        vy_predict[k] = vy[k - 1] + (accel_ay[k] * delta_t[k])

        p11[k] = p11[k-1] + q11; p12[k] = 0
        p21[k] = 0             ; p22[k] = p22[k-1] + q22

        #update
        g11 = (p11[k]) / (p11[k] + r11); g12 = 0
        g21 = 0                        ; g22 = (p22[k]) / (p22[k] + r22)

        vx[k] = vx_predict[k] + g11 * (flow_vx[k] - vx_predict[k])
        vy[k] = vy_predict[k] + g22 * (flow_vy[k] - vy_predict[k])

        p11[k] = (1 - g11) * p11[k]; p12[k] = 0
        p21[k] = 0                 ; p22[k] = (1 - g22) * p22[k]

kalman_filter()

#plot result
plt.figure('Kalman Filter - x')
plt.plot(time, flow_vx)
plt.plot(time, accel_ax)
plt.plot(time, vx)
plt.legend(['flow_vx', 'accel_ax', 'kalman_vx'], loc='upper left')

plt.figure('Kalman Filter - y')
plt.plot(time, flow_vy)
plt.plot(time, accel_ay)
plt.plot(time, vy)
plt.legend(['flow_vy', 'accel_ay', 'kalman_vy'], loc='upper left')

plt.show()
