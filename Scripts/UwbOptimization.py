# Created by steve on 17-12-19 上午10:45
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

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from Scripts.ViewerModel import PF_FRAME

from Scripts.ViewerModel import TranglePose
import time

if __name__ == '__main__':
    dir_name = '/home/steve/Data/NewIU/17/'
    beaconset = np.loadtxt(dir_name + 'beaconset.csv', delimiter=',')
    uwb_data = np.loadtxt(dir_name + 'uwb_result.csv', delimiter=',')

    ## only particle filter
    filter_start = time.time()
    particle_num = 15
    sample_sigma = 1.5

    pf = PF_FRAME.PF_Frame([2000, 2000],
                           [1000, 1000],
                           10,
                           particle_num)
    pf.SetBeaconSet(beaconset[:, 0:2])
    pf.InitialPose([0.0, 0.0])

    only_pf_result = np.zeros([uwb_data.shape[0], 2])

    for i in range(uwb_data.shape[0]):
        pf.Sample(sample_sigma)
        pf.Evaluated(uwb_data[i, 1:])
        pf.ReSample()

        only_pf_result[i, :] = pf.GetResult()
    filter_end = time.time()
    print('cost time :', filter_end - filter_start)

    tp = TranglePose.trianglepose(beaconset, uwb_data[:, 1:])
    op_result = tp.ComputePath(uwb_data[:, :])
    op_result = op_result[:, 1:]
    np.savetxt('/home/steve/Code/QuickFusing/ResultData/optimized_res.txt',
               op_result)

    plt.figure()
    plt.plot(only_pf_result[:, 0], only_pf_result[:, 1], label='only uwb pf')
    plt.plot(op_result[:, 0], op_result[:, 1], label='optimized result')
    plt.legend()
    plt.grid()
    plt.show()
