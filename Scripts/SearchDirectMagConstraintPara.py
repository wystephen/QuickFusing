# Created by steve on 17-11-16 下午5:35
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

import sys
import os
import time
import random

import numpy as np

import threading


def run_the_str(cmd_str, count):
    print(str(count) + 'started')
    # time.sleep(int(random.random()*3))
    os.system(cmd_str)
    print(str(count) + ' finished.')


if __name__ == '__main__':

    all_str_run = list()

    para_list = [-1.0, 0.001, 0.002, 0.005, 0.01, 0.03, 0.06, 0.08, 0.1, 0.3, 0.6, 0.9, 1.3, 1.6, 2.1, 3.3, 4.6, 8.0,
                 10, 20]

    for i1 in para_list:
        for i2 in para_list:
            for i3 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
                all_str_run.append("../cmake-build-debug/DirectMagConstraint 100.0 100.0 {0} {1} {2}".format(
                    i1,
                    i2,
                    i3 / 2.0
                ))

    print("finished preparing parameters!")
    print('all count: ', len(all_str_run))

    random.shuffle(all_str_run)
    print('reshuffled list.')

    count = 0
    last_update_time = -1
    while count < len(all_str_run):
        # print(threading.active_count())
        time.sleep(1.5)

        para = np.loadtxt('ParaSet.txt')
        print(para)


        if threading.active_count() < para:
            t = threading.Thread(target=run_the_str, args=(all_str_run[count], count))
            count += 1
            t.start()
