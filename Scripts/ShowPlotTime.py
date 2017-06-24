# Created by steve on 17-6-24 上午10:14



import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/67/"

    imu_time = np.loadtxt(dir_name+'imu.txt',delimiter=',')
    uwb_data = np.loadtxt(dir_name+'uwb_result.csv',delimiter=',')

    print(min(imu_time[:,0]),max(imu_time[:,0]))
    print(min(uwb_data[:,0]),max(uwb_data[:,0]))


    plt.figure()
    plt.plot(imu_time[:,0],'r')
    plt.plot(uwb_data[:,0],'b')

    plt.show()

