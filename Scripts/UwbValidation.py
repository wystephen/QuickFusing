# -*- coding:utf-8 -*-
# Create by steve in 17-4-27 at 上午8:03

import scipy as sp
import scipy.optimize as optimize

import numpy as np
import matplotlib.pyplot as plt


class beaconData:
    def __init__(self,beaconset,b_dist):
        self.beaconset = beaconset
        self.b_dist = b_dist


    def sum_error(self,pose):
        # print(self.beaconset.shape)
        # print(self.b_dist.shape)
        # print(self.beaconset-pose)

        error = np.sum(np.abs(np.sum((self.beaconset-pose)**2.0,axis=1)**0.5 - self.b_dist))

        # if(pose[2]<1.2) or (pose[2]>2.2):
        #     error*=10.0
        # print(error)
        # if(pose[])
        return error







if __name__ == '__main__':
    dir_name = "/home/steve/locate/3"
    uwb_data = np.loadtxt(dir_name+"UwbData.data.csv",delimiter=',')
    beacon = np.loadtxt(dir_name+"beaconset.data.csv",delimiter=',')

    uwb_real = np.loadtxt(dir_name+"UwbRealPose.data.csv",delimiter=',')

    res_err = np.zeros([uwb_data.shape[0]])

    all_res = np.zeros([uwb_data.shape[0],3])

    # optimize.fmin_bfgs()
    last_res = uwb_real[0,:]
    for i in range(uwb_data.shape[0]):
        bd = beaconData(beaconset=beacon,b_dist=uwb_data[i,1:])
        # bd.sum_error([0.0,0.0,0.0])
        if last_res[2]<1.2 or last_res[2] > 2.2:
            last_res[2] = 1.8
        res = optimize.fmin_bfgs(bd.sum_error,last_res)
        print("----------------------1")
        print((res))
        print("----------------------2")
        res_err[i] = bd.sum_error(res)
        last_res = res
        all_res[i,:] = res



    np.savetxt(dir_name+"UwbValid.data.csv",res_err,delimiter=',')


    plt.figure()
    plt.grid(True)
    for i in range(1,uwb_data.shape[1]):
        plt.plot(uwb_data[:,i])
    plt.plot(res_err,label='res err')
    plt.legend()


    plt.figure()
    plt.grid(True)
    plt.plot(all_res[:,0],all_res[:,1],'r-')
    plt.plot(uwb_real[:,0],uwb_real[:,1],'b-+')

    plt.show()