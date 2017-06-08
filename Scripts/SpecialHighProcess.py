# -*- coding:utf-8 -*-
# Create by steve in 17-6-8 at 上午9:19


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt


if __name__ == '__main__':
    dir_name = "/home/steve/Code/Mini-IMU/Scripts/IMUWB/46/"
    high_mat = np.loadtxt(dir_name+'vertex_high.csv',delimiter=',')

    print(high_mat.shape)

    stat_whole = np.zeros_like(high_mat)
    stat_whole += np.min(high_mat)

    diff_whole = np.zeros_like(high_mat)
    # diff_whole = 0.0
    diff_whole[2:] = np.abs(high_mat[2:]-high_mat[:-2])

    for i in range(10,high_mat.shape[0]):
        if np.abs(high_mat[i]-high_mat[i-10])>1e5:
            stat_whole[i] = np.max(high_mat)

    plt.figure()
    plt.plot(high_mat)
    plt.plot(stat_whole)


    # plt.figure()
    # plt.plot(diff_whole)
    plt.show()