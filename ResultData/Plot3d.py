import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

import os

if __name__ == '__main__':
    trace_data = list()
    trace_name = list()

    # if o1
    imu = np.loadtxt('./imu.txt')
    trace_data.append(imu)
    trace_name.append('graph-optimized')

    # plot by list......
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(len(trace_data)):
        data = trace_data[i]
        name = trace_name[i]
        print(name)

        ax.plot(data[:, 0], data[:, 1], data[:, 2],
                '*-',
                label=name)

    plt.grid()
    plt.legend()
    plt.show()
