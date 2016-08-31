import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
import math

file_name = raw_input(">> ")

# Read data
data = genfromtxt(file_name, delimiter=(','))


ground_x = data[:, 0]
ground_y = data[:, 1]
ground_x = [x - ground_x[0] for x in ground_x]
ground_y = [y - ground_y[0] for y in ground_y]
ground_x = np.array(ground_x)
ground_y = np.array(ground_y)

roll = []
pitch = []
yaw = []

if False:
    for i in range(0, len(data[:, 0])):
        roll.append(math.atan2((2 * ((data[i, 3] * data[i, 4]) + (data[i, 5] *
            data[i, 6]))), (1 - 2 * ((data[i, 4] * data[i, 4]) + (data[i, 5])))) /
            math.pi * 180)
        print math.asin(2 * (data[i, 3] * data[i, 5] - data[i, 6] * data[i, 4])) / math.pi * 180
        pitch.append(math.asin(2 * (data[i, 3] * data[i, 5] - data[i, 6] * data[i,
            4])) / math.pi * 180)
        yaw.append(math.atan2(2 * (data[i, 3] * data[i, 6] + data[i, 4] * data[i,
            5]), 1 - 2 * (data[i, 5] * data[i, 5] + data[i, 6] * data[i, 6])) /
            math.pi * 180)

estimated_x = data[:, 8]
estimated_y = data[:, 7]
estimated_x = np.array(estimated_x)
estimated_y = np.array(estimated_y)

diff = np.sqrt(np.square(ground_x - estimated_x) + np.square(ground_y -
    estimated_y))

# print roll
# print pitch
# print yaw

# plt.subplot(211)
# plt.plot(ground_x)
# plt.plot(estimated_x)

# plt.subplot(212)
# plt.plot(ground_y)
# plt.plot(estimated_y)

# plt.subplot(213)
# plt.plot(ground_x, ground_y)
# plt.plot(estimated_x, estimated_y)

# plt.plot(roll)
# plt.plot(pitch)
# plt.plot(yaw)
# plt.plot(data[:, 11])

plt.plot(diff)

plt.show()
