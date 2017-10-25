# Created by steve on 17-10-25 下午7:57
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

import threading


def run_the_str(str):
    print(str + 'started')
    # time.sleep(int(random.random()*3))
    print(str + ' finished.')


if __name__ == '__main__':

    all_str_run = list()
    for i1 in [0.00001, 0.00004, 0.0001, 0.0004, 0.0056, 0.001, 0.011, 0.05, 0.1, 0.3, 0.9, 1.2]:
        for i2 in [0.00001, 0.0001, 0.008, 0.02, 0.05, 0.1, 0.9, 1.0]:
            for i3 in [0.001, 0.003, 0.007, 0.01, 0.03, 0.08, 0.3, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0]:
                all_str_run.append("../cmake-build-debug/JYImuMag {0} {1} {2} ".format(
                    i1, i2, i3

                ))
    print("finished preparing parameters!")

    count = 0
    while count < len(all_str_run):
        # print(threading.active_count())
        if threading.active_count() < 12:
            t = threading.Thread(target=run_the_str, args=(all_str_run[count],))
            count += 1
            t.start()
