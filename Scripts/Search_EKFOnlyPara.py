# Created by steve on 17-11-9 下午7:36
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
import scipy as sp
import threading


def run_the_str(cmd_str,count):
    print(str(count) + 'started')
    # time.sleep(int(random.random()*3))
    os.system(cmd_str)
    print(str(count) + ' finished.')


if __name__ == '__main__':
    para_list = list()

    for first_info in [0.0001,0.0003,0.0008,0.001,0.003,0.005,0.01,0.02,0.04,0.05,0.1,0.3,0.5,0.8,1.0,1.2,1.5,2.0,2.5,5.0,7.0,10.0,20,30,40.0,60.0,70.0]:
        for second_info in [0.0001,0.0003,0.0008,0.001,0.003,0.005,0.01,0.02,0.04,0.05,0.1,0.3,0.5,0.8,1.0,1.2,1.5,2.0,2.5,5.0,7.0,10.0,20,30,40.0,60.0,70.0]:
            for ori_info in [0.001,0.005,0.01,0.05,0.1,0.5,1.0,1.5,2.0,2.5,5.0,7.0,10.0,40.0,60.0,70.0]:
                para_list.append("../cmake-build-debug/EKFOnly {0} {1} {2} ".format(
                    first_info,
                    second_info,
                    ori_info
                ))

    import random
    random.shuffle(para_list)


    count = 0
while count < len(para_list):
    # print(threading.active_count())
    if threading.active_count() < 7:
        t = threading.Thread(target=run_the_str, args=(para_list[count],count))
        count += 1
        t.start()