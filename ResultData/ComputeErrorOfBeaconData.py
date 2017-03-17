import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

from array import array
import os


if __name__ == '__main__':
    dir_name = '/home/steve/locate/'

    range_array=array("d")
    dis_array = array("d")

    for name in os.listdir(dir_name):
        if 'UwbData.data.csv' in name:
            print(int(name[0]))
            id_name = int(name[0])
            if not id_name==5:
                continue
            beacon_set = dir_name+'{0}beaconset.data'.format(id_name)
            real_pose = dir_name +'{0}UwbRealPose.data.csv'.format(id_name)
            uwb_data = dir_name +'{0}UwbData.data'.format(id_name)

            beacon_set = np.loadtxt(beacon_set)
            real_pose = np.loadtxt(real_pose,delimiter=',')
            uwb_data = np.loadtxt(uwb_data)


            for i in range(real_pose.shape[0]):
                real_dist = np.sum((real_pose[i,:]-beacon_set)**2.0,1)**0.5

                # print(real_dist.shape)
                for k in range(real_dist.shape[0]):
                    dis_array.append(real_dist[k])
                    range_array.append(uwb_data[i,k+1])
                    print(uwb_data[i,k+1])



    uwb_range = np.frombuffer(range_array,dtype=np.float).reshape(-1,4)
    real_range = np.frombuffer(dis_array,dtype=np.float).reshape(-1,4)
    # offset = 18
    # real_range[:-offset,:] = real_range[offset:,:]


    # reshape and sort

    # uwb_range = uwb_range.reshape(-1)
    # real_range = real_range.reshape(-1)
    # sort_list = np.argsort(real_range)
    # real_range  = real_range[sort_list]
    # uwb_range = uwb_range[sort_list]

    print(uwb_range.shape,real_range.shape)
    np.savetxt(dir_name+"real_range",real_range)
    np.savetxt(dir_name+"uwb_range",uwb_range)

    plt.figure(0)
    plt.plot(uwb_range[:],'r',label='UwbRange')
    plt.plot(real_range[:],'b',label = 'real range')
    plt.plot((uwb_range-real_range)[:],'g',label = 'offset')
    plt.legend()
    plt.grid(True)
    plt.show()



