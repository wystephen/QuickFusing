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

    print(uwb_range.shape,real_range.shape)

    plt.figure(0)
    plt.plot(uwb_range[:,2],'r')
    plt.plot(real_range[:,2],'b')
    plt.plot((uwb_range-real_range)[:,2],'g')
    plt.grid(True)
    plt.show()



