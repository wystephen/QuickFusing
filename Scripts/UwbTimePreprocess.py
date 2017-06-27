# Created by steve on 17-6-24 上午10:40


import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/67/"

    imu_time = np.loadtxt(dir_name+'imu.txt',delimiter=',')
    uwb_data = np.loadtxt(dir_name+'uwb_result.csv',delimiter=',')


    time_base = 0.5 * (min(imu_time[:,0])+min(uwb_data[:,0]))

    time_inteval_factor = (max(imu_time[:,0])-time_base) / (max(uwb_data[:,0])-time_base)

    # for i in range(uwb_data.shape[0]):
    #     uwb_data[i,0] = time_base+(uwb_data[i,0]-time_base)*time_inteval_factor

    print(min(imu_time[:,0]),max(imu_time[:,0]))
    print(min(uwb_data[:,0]),max(uwb_data[:,0]))

    np.savetxt(dir_name+'uwb_result.csv',uwb_data,delimiter=',')
    #
    # plt.figure()
    # plt.plot(imu_time[:,0],'r')
    # plt.plot(uwb_data[:,0],'b')
    #
    # plt.show()
