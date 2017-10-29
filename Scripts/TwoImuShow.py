# Created by steve on 17-10-29 下午4:53
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


import Scripts.ImuResultReader
import Scripts.ImuPreprocess
import Scripts.PcSavedReader

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

import os

if __name__ == '__main__':
    # dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/73/"
    dir_name = "/home/steve/Data/II/18/"
    # dir_name = "/home/steve/Code/Mini-IMU/Scripts/IMUWB/20/"
    # dir_name = "/home/steve/Data/FastUwbDemo/2/"
    # dir_name = "/home/steve/tmp/test/20/"


    # a = np.loadtxt(dir_name + 'imu.txt', delimiter=',')
    # print(a[:, 1:4].shape)
    # a[:,1:4] *= 9.816


    ip = Scripts.ImuPreprocess.ImuPreprocess(dir_name + "imu.txt")
    ip.computezupt()
    # plt.show()
    ip.findvertex()
    # print(ip.zupt_result)
    #
    # np.savetxt(dir_name + "sim_pose.csv", ip.vertics, delimiter=',')
    # np.savetxt(dir_name + "all_quat.csv", ip.vertex_quat, delimiter=',')
    # np.savetxt(dir_name + "sim_zupt.csv", ip.zupt_result, delimiter=',')
    # np.savetxt(dir_name + "vertex_time.csv", ip.vertics_time, delimiter=",")
    # np.savetxt(dir_name + "vertex_high.csv", ip.vertics_high, delimiter=',')

    # print(ip.vertics.shape, " - ", ip.vertex_quat.shape, " - ", ip.vertics_time.shape)
    ip2 = Scripts.ImuPreprocess.ImuPreprocess(dir_name+'imu2.txt')
    ip2.computezupt()
    ip2.findvertex()

    trace_fig = plt.figure()
    ax = trace_fig.gca(projection='3d')
    ax.plot(ip.vertics[:, 0], ip.vertics[:, 1], ip.vertics[:, 2],
            'r*-',
            label='trace imu.txt')
    ax.plot(ip2.vertics[:,0],ip2.vertics[:,1],ip2.vertics[:,2],
            'b*-',
            label='trace imu2.txt')
    ax.legend()


    # plt.figure()
    # plt.plot(ip.vertics_time, 'r')

    # ip.findcorner()
    # ip.computeconerfeature()
    plt.show()