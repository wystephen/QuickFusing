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
from scipy.optimize import minimize
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from Scripts.EllipsoidEq import EllipsoidEq

if __name__ == '__main__':
    # imu_data = np.loadtxt('/home/steve/Code/Mini_IMU/Scripts/IMUWB/91/imu.txt',
    #                       delimiter=',')
    imu_data = np.loadtxt('/home/steve/Data/IU/86/imu.txt',
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
    plt.title('mag normalized direct x y z')
    plt.grid()
    for i in range(7, imu_data.shape[1] - 1):
        plt.plot(imu_data[:, i] / np.linalg.norm(imu_data[:, 7:10], axis=1),
                 '.-',
                 label='mag:' + str(i - 6))
    plt.legend()

    fig = plt.figure()
    ax = Axes3D(fig)
    # ax.scatter(imu_data[:,7]/np.linalg.norm(imu_data[:, 7:10], axis=1),
    #         imu_data[:,8]/np.linalg.norm(imu_data[:, 7:10], axis=1),
    #         imu_data[:,9]/np.linalg.norm(imu_data[:, 7:10], axis=1))

    ax.scatter(imu_data[:, 7],
               imu_data[:, 8],
               imu_data[:, 9])
    mag_norm = np.linalg.norm(imu_data[:, 7:10], axis=1)

    e_equation = EllipsoidEq(imu_data[:, 7],  # / mag_norm,
                             imu_data[:, 8],  # @ / mag_norm,
                             imu_data[:, 9])  # / mag_norm)

    mag_central = [-25.0, -128.0, 80.0]
    mag_scale = [238.0, 263.0, 271.0]

    all_mag = (imu_data[:, 7:10] - mag_central) / mag_scale

    fig_normed_mag = plt.figure()
    ax_normed_mag = Axes3D(fig_normed_mag)

    ax_normed_mag.plot(all_mag[:, 0], all_mag[:, 1], all_mag[:, 2], '.y')

    plt.figure()
    plt.title('normed mag')
    for i in range(all_mag.shape[1]):
        plt.plot(all_mag[:, i], '-+', label=str(i))
    plt.plot(np.linalg.norm(all_mag, axis=1), label='norm')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.title('really normed mag')

    div_norm = np.linalg.norm(all_mag, axis=1)
    for i in range(all_mag.shape[1]):
        all_mag[:, i] /= div_norm
        plt.plot(all_mag[:, i], '-+', label=str(i))
    plt.legend()
    plt.grid()

    # print('before error:', e_equation.errorFunction([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    # res_x = minimize(e_equation.errorFunction, x0=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], method='L-BFGS-B',
    #                  bounds=[
    #                      [0, 1000],
    #                      [0, 1000],
    #                      [0, 1000],
    #                      [0, 1000],
    #                      [0, 1000],
    #                      [0, 1000]
    #                  ])
    # print(res_x)
    # tx = res_x.x
    # print(res_x.x)
    # ax.contour(imu_data[:,8],imu_data[:,])
    # print('after error:', e_equation.errorFunction(tx))
    # print('central x,y:', (np.max(imu_data[:, 7:10], axis=0) - np.min(imu_data[:, 7:10], axis=0)) / 2)
    # print('central x y z :', tx[0], tx[2], tx[4])
    ### The last moment for compute...
    # for imu mag
    #    central =
    #
    #            -25  -128    80
    #
    #
    #    Scale_axis =
    #
    #            238   263   271

    plt.figure()
    plt.grid()

    plt.title('all compara with mag z')
    for i in range(7,imu_data.shape[1]-1):
        plt.plot(imu_data[:,i]/imu_data[:,9],'-+',label=str(i-6))
    plt.legend()

    plt.show()
