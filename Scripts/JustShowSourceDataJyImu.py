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


    acc_cent = [0.0195,0.0154,-0.0877]
    acc_scale =[ 1.0015,1.0008,1.0336]

    plt.figure()
    plt.title("acc")
    for i in range(3):
        plt.plot((imudata[:,i+1]-acc_cent[i])/acc_scale[i],'*-',label=str(i))

    plt.plot(np.linalg.norm((imudata[:,1:4]-acc_cent)/acc_scale,axis=1),'r-',label="norm")
    plt.grid()
    plt.legend()




    plt.figure()
    plt.title("gyro")
    for i in range(3):
        plt.plot(imudata[:,i+4],'*-',label=str(i))
    plt.grid()
    plt.legend()


    mag_cent = [-58.0512,-117.0970,151.9004]
    mag_scale = [213.8826,208.3894,232.3945]


    plt.figure()
    plt.title("mag")
    for i in range(3):
        plt.plot((imudata[:,i+7]-mag_cent[i])/mag_scale[i],'*-',label=str(i))

    plt.plot(np.linalg.norm((imudata[:,7:10]-mag_cent)/mag_scale,axis=1),'r-',label="norm")
    plt.grid()
    plt.legend()
    plt.show()
