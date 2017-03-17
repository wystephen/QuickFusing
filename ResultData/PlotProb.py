import numpy as np
import scipy as sp


import matplotlib.pyplot as plt

import os

def NormalPdf(x,miu,sigma):
    para1 = 1/ (2.0*np.pi)**0.5 / sigma
    para2 = -((x-miu)*(x-miu)/2/sigma/sigma)
    return para1 * np.exp(para2)

def EvaluationFunction(pose,uwb_data,beaconset,sigma,z_offset,save_file_name):
    the_range = 3*sigma
    r_k = 10.0
    plot_pose = [the_range*r_k,the_range*r_k]


    out_matrix = np.zeros([int(the_range*2*r_k),int(the_range*2*r_k)])
    print(range(out_matrix.shape[0]))
    for i in range(out_matrix.shape[0]):
        for j in range(out_matrix.shape[1]):
            tx = float(i) / r_k - the_range + pose[0]
            ty = float(j) / r_k -the_range +pose[1]

            score = 1.0
            for k in range(beaconset.shape[0]):
                dist = np.linalg.norm([tx,ty,z_offset]-beaconset[k,:])
                score *= (NormalPdf(uwb_data[k],dist,sigma)+1e-5)
            out_matrix[i,j] = score

    plt.figure(2)
    plt.contourf(out_matrix)
    plt.plot(plot_pose[0],plot_pose[1],'Dr')
    # plt.xlim(-)
    plt.savefig(save_file_name)
    # plt.show()
    # plt.plot()
    return out_matrix



if __name__ == '__main__':

    dir_name = '/home/steve/locate/'
    z_offset = 1.8
    beaconset = np.loadtxt(dir_name+'5beaconset.data')
    uwbdata = np.loadtxt(dir_name+'5UwbData.data')
    real_pose = np.loadtxt(dir_name+'5UwbRealPose.data.csv',delimiter=',')

    save_dir = '/home/steve/locate/SaveProb'
    # if not os.listdir(save_dir):
    #     print("error")
    os.mkdir(save_dir)

    print(real_pose.shape,uwbdata.shape)

    # plot_index = 227
    # m = EvaluationFunction(real_pose[plot_index,:],uwbdata[plot_index,:],beaconset,2.0,z_offset)
    for i in range(uwbdata.shape[0]):
        EvaluationFunction(real_pose[i,:2],
                           uwbdata[i,:],
                           beaconset,
                           2.0,
                           z_offset,
                           save_dir+"/{0}.jpg".format(i))





