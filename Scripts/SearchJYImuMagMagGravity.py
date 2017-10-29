# Created by steve on 17-10-28 下午5:05
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


def run_the_str(cmd_str, count):
    print(str(count) + 'started')
    # time.sleep(int(random.random()*3))
    os.system(cmd_str)
    print(str(count) + ' finished.')


if __name__ == '__main__':

    all_str_run = list()
    for i1 in [0.0000001]:
        for i2 in [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 3.0, 8.0]:
            for i3 in [0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 3.0, 8.0]:
                for i4 in [-9.8]:
                    for i5 in [-1.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5,2.0,3.0,4.0]:
                        for i6 in [-1.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5,2.0,3.0,4.0]:
                            for i7 in [0.0]:
                                all_str_run.append("../cmake-build-debug/JYImuMag {0} {1} {2} {3} {4} {5} {6} ".format(
                                    i1, i2, i3, i4, i5, i6, i7

                                ))
    # for i1 in [0.0000001]:
    #     for i2 in [0.5]:
    #         for i3 in [0.5]:
    #             for i4 in [-9.8]:
    #                 for i5 in [-1.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 1.8, 2.1,
    #                            2.7, 3.0, 3.3, 4.0]:
    #                     for i6 in [-1.0, 0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 1.8,
    #                                2.1, 2.7, 3.0, 3.3, 4.0]:
    #                         for i7 in [0.0]:
    #                             all_str_run.append("../cmake-build-debug/JYImuMag {0} {1} {2} {3} {4} {5} {6} ".format(
    #                                 i1, i2, i3, i4, i5, i6, i7
    #
    #                             ))

    print("finished preparing parameters!")
    print('all count: ', len(all_str_run))

    random.shuffle(all_str_run)
    print('reshuffled list.')

    count = 0
    while count < len(all_str_run):
        # print(threading.active_count())
        time.sleep(0.5)
        if threading.active_count() < 7:
            t = threading.Thread(target=run_the_str, args=(all_str_run[count], count))
            count += 1
            t.start()
