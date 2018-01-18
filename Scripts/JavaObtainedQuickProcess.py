# Created by steve on 18-1-9 上午9:19
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

import ImuResultReader
import ImuPreprocess
import PcSavedReader

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

import os
from UwbDataPreprocess import UwbDataPre

import JavaObtainedUWBProcess

if __name__ == '__main__':
    # dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/73/"
    dir_name = "/home/steve/Data/FusingLocationData/0015/"

    a = np.loadtxt(dir_name + 'LEFT_FOOT.data', delimiter=',')
    a = a[:, 1:]

    np.savetxt(dir_name + "sim_imu.csv", a, delimiter=',')

    ip = ImuPreprocess.ImuPreprocess(dir_name + "sim_imu.csv")
    ip.computezupt()
    # plt.show()
    ip.findvertex()
    # print(ip.zupt_result)

    np.savetxt(dir_name + "sim_pose.csv", ip.vertics, delimiter=',')
    np.savetxt(dir_name + "all_quat.csv", ip.vertex_quat, delimiter=',')
    np.savetxt(dir_name + "sim_zupt.csv", ip.zupt_result, delimiter=',')
    np.savetxt(dir_name + "vertex_time.csv", ip.vertics_time, delimiter=",")
    np.savetxt(dir_name + "vertex_high.csv", ip.vertics_high, delimiter=',')
    #
    print(ip.vertics.shape, " - ", ip.vertex_quat.shape, " - ", ip.vertics_time.shape)

    trace_fig = plt.figure()
    ax = trace_fig.gca(projection='3d')
    ax.plot(ip.vertics[:, 0], ip.vertics[:, 1], ip.vertics[:, 2],
            'r*-',
            label='trace')
    ax.legend()

    u = JavaObtainedUWBProcess.UwbProcess(dir_name + 'HEAD_UWB.data', dir_name + '../mac.txt')
    np.savetxt(dir_name + 'uwb_result.csv', u.uwb_data, '%.19f', delimiter=',')

    plt.show()
