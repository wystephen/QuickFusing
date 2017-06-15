# -*- coding:utf-8 -*-
# Create by steve in 17-6-8 at 上午9:19


import numpy as np
import scipy as sp

import matplotlib.pyplot as plt


if __name__ == '__main__':
    dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/46/"
    high_mat = np.loadtxt(dir_name+'vertex_high.csv',delimiter=',')


    para_high = 4.0

    out_high = np.zeros_like(high_mat)

    print(high_mat.shape)

    stat_whole = np.zeros_like(high_mat)
    stat_whole += np.min(high_mat)

    diff_whole = np.zeros_like(high_mat)
    # diff_whole = 0.0
    diff_whole[2:] = np.abs(high_mat[2:]-high_mat[:-2])



    avg_press = np.mean(high_mat)

    for i in range(20,high_mat.shape[0]-20):
        if np.abs(high_mat[i]-high_mat[i-14])>5e9 or\
                        np.abs(high_mat[i]-high_mat[i+14])>5e9:
            stat_whole[i-2:i+2] = np.max(high_mat)

    for i in range(high_mat.shape[0]):
        if stat_whole[i] < avg_press:
            if high_mat[i] < avg_press:
                out_high[i] = 0.0#para_high
            else:
                out_high[i] = para_high#.0

        else:
            out_high[i]=-10

    # out_high *= 0.0


    plt.figure()
    plt.plot(high_mat,'*-')
    plt.plot(stat_whole)


    # plt.figure()
    # plt.title('OUT high')
    # plt.plot(out_high)


    np.savetxt(dir_name+'vertex_high_modified.csv',out_high,delimiter=',')


    # plt.figure()
    # plt.plot(diff_whole)
    plt.show()