# Created by steve on 17-12-18 下午5:33
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

import os
import re
import numpy as np
import matplotlib.pyplot as plt
import array


class NewUwbDataPre:

    def __init__(self, data_file_dir, mac_pose_file_name):
        '''

        :param data_file_dir: directory of data file.

        :param mac_pose_file_name: i.e. /dir/to/file/file_name.csv
        '''
        print(self.__class__)
        mac_pose_file = open(mac_pose_file_name)
        self.mac_dic = dict()
        for index, line in enumerate(mac_pose_file.readlines()):
            self.mac_dic[line.split(',')[0]] = \
                (int(index),
                 float(line.split(',')[1]),
                 float(line.split(',')[2]))

        for key in self.mac_dic:
            print(key,
                  self.mac_dic.get(key)[0],
                  self.mac_dic.get(key)[1],
                  self.mac_dic.get(key)[2])


if __name__ == '__main__':
    dir_name = '/home/steve/Data/NewIU/'
    num_data = 11

    # mac_pose_file = open(dir_name + 'MacPose.csv')
    # mac_dic = dict()
    # for index,line in enumerate(mac_pose_file.readlines()):
    #     mac_dic[line.split(',')[0]] = (int(index),float(line.split(',')[1]), float(line.split(',')[2]))
    #
    # for key in mac_dic:
    #     print(key, mac_dic.get(key)[0], mac_dic.get(key)[1])
    nudp = NewUwbDataPre(dir_name + str(num_data) + '/', dir_name + 'MacPose.csv')
