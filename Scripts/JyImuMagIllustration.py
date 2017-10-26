# Created by steve on 17-10-26 下午7:04
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
    imu_data = np.loadtxt('/home/steve/Code/Mini_IMU/Scripts/IMUWB/91/imu.txt',
                          delimiter=',')

    plt.figure()
    plt.title('mag x y z ')
    plt.grid()

    for i in range(7, imu_data.shape[1] - 1):
        plt.plot(imu_data[:, i], '.-', label='mag:' + str(i - 6))
    plt.legend()

    plt.figure()
    plt.title('mag norm')
    plt.grid()
    # print('imu shape',imu_data[:,7:10].shape)
    plt.plot(np.linalg.norm(imu_data[:, 7:10], axis=1))

    plt.figure()
    plt.title('mag normalized x y z')
    plt.grid()
    for i in range(7, imu_data.shape[1] - 1):
        plt.plot(imu_data[:, i] / np.linalg.norm(imu_data[:, 7:10], axis=1),
                 '.-',
                 label='mag:' + str(i - 6))
    plt.legend()



    ### The last moment for compute...

    plt.show()
