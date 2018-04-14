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

    for first_info in [0.1,0.3,0.5,0.7,1.0,1.4,1.8,2.5,3.0,5.0,6.0,7.0,8.0]:
        for second_info in [0.1,0.3,0.5,0.7,1.0,1.4,1.8,2.5,3.0,5.0,6.0,7.0,8.0]:
            for ori_info in [1.0,5.0,20.0,100]:
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