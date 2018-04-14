# Created by steve on 17-6-13 上午9:24
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import time
import os


class RealTimePlot:
    def __init__(self, file_name):
        self.file_name = file_name
        self.n_time = os.stat(file_name).st_mtime

        self.fig = plt.figure()
        self.ax = p3.Axes3D(self.fig)

        self.data = np.loadtxt(self.file_name)
        self.ax.plot(self.data[:, 0], self.data[:, 1], self.data[:, 2], 'r*-')

    def start(self):
        line_ani = animation.FuncAnimation(
            self.fig, self.update_trace, interval=100
        )
        plt.show()

    def update_trace(self, data):
        if (os.stat(file_name).st_mtime > self.n_time):
            time.sleep(1)
            self.n_time = os.stat(self.file_name).st_mtime
            self.data = np.loadtxt(self.file_name)
            self.ax.clear()

            self.ax.plot(self.data[:, 0], self.data[:, 1], self.data[:, 2], 'r*-')


if __name__ == '__main__':
    file_name = '../ResultData/imu.txt'
    # print(os.stat(file_name).st_mtime)
    rshow = RealTimePlot(file_name)

    rshow.start()
