# Created by steve on 17-10-14 下午8:19
'''
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
'''

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # dir_name = "/home/steve/Data/XIMU&UWB/5/"
    dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/91/"

    # imu_data = np.loadtxt(dir_name + "ImuData.csv",delimiter=',')
    imu_data = np.loadtxt(dir_name + "imu.txt", delimiter=',')
    print(imu_data.shape)

    plt.figure()
    for i in range(3):
        plt.plot(imu_data[:,i+6]/np.pi * 180.0,'-')

    plt.figure()
    plt.plot(np.linalg.norm(imu_data[:,6:10],axis=1))
    # plt.plot(imu_data[:,12]*180.0 ,'-')

    # plt.figure()
    # plt.plot(imu_data[:, 2], 'b-')
    # plt.plot(imu_data[:, 9] * 180.0, 'r+-')

    # hist

    plt.figure()
    plt.hist(imu_data[:, 9], 60)

    plt.show()
