# Created by steve on 17-6-15 上午10:46

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt


if __name__ == '__main__':
    imu_res =  np.loadtxt('../ResultData/imu.txt')
    uwb_res = np.loadtxt("../ResultData/uwb.txt")

    dist_all = np.zeros([imu_res.shape[0],uwb_res.shape[0]])



    plt.figure()
    for i in range(uwb_res.shape[0]):
        dist_all[:,i] = np.linalg.norm((imu_res-uwb_res[i,:]),axis=1)
        plt.plot(dist_all[:,i],'-',label=str(i))
    plt.legend()
    plt.show()


