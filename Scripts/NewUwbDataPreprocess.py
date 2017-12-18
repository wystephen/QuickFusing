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
    def __init__(self):
        print(self.__class__)



if __name__ == '__main__':
    dir_name = '/home/steve/Data/NewIU/'
    num_data = 10


    mac_pose_file = open(dir_name+'MacPose.csv')
    mac_dic  = dict()
    for line  in mac_pose_file.readlines():
        mac_dic[line.split(',')[0]] = (float(line.split(',')[1]),float(line.split(',')[2]))

    for key in mac_dic:
        print(key,mac_dic.get(key))



