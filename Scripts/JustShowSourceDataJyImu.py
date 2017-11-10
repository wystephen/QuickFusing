# Created by steve on 17-11-9 下午3:05
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


import matplotlib.pyplot as plt
import scipy as sp
import numpy as np

if __name__ == '__main__':
    dir_nema = '/home/steve/Data/II/16/'
    imudata = np.loadtxt(dir_nema+'imu2.txt',delimiter=',')

    plt.figure()
    plt.title("acc")
    for i in range(3):
        plt.plot(imudata[:,i+1],'*-',label=str(i))
    plt.grid()
    plt.legend()



    plt.figure()
    plt.title("gyro")
    for i in range(3):
        plt.plot(imudata[:,i+4],'*-',label=str(i))
    plt.grid()
    plt.legend()


    plt.figure()
    plt.title("mag")
    for i in range(3):
        plt.plot(imudata[:,i+7],'*-',label=str(i))
    plt.grid()
    plt.legend()


    plt.show()
