import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

if __name__ == '__main__':
    all_result = np.loadtxt("5all_result.data")

    print(all_result)
    sort_order = np.argsort(all_result[:, 0])
    all_result = all_result[sort_order, :]

    select_list = list()
    for i in range(all_result.shape[0]):
        # if np.abs(all_result[i, 1] - 2.0) < 1e-5 and np.abs(all_result[i, 2] - 1.0) < 1e-5:
            select_list.append(i)

    plt.figure(1)
    plt.grid(True)

    plt.plot(all_result[select_list, 0], all_result[select_list, 4], 'r.-', label='fuse err')
    # plt.plot(all_result[select_list,0],all_result[select_list,4]+all_result[select_list,5],'r-.')
    # plt.plot(all_result[select_list,0],all_result[select_list,4]-all_result[select_list,5],'r-.')

    # plt.plot(all_result[select_list,0],all_result[select_list,5],'r',label = 'fus std')
    plt.plot(all_result[select_list, 0], all_result[select_list, 3], 'b-', label='uwb err')
    plt.legend()

    plt.figure(2)
    plt.grid(True)
    plt.plot(all_result[select_list, 0], all_result[select_list, 7], 'r.-', label='fuse time')
    plt.plot(all_result[select_list, 0], all_result[select_list, 7] + all_result[select_list, 8], 'r-.')
    plt.plot(all_result[select_list, 0], all_result[select_list, 7] - all_result[select_list, 8], 'r-.')
    # plt.plot(all_result[select_list,0],all_result[select_list,8],'b-.',label='fuse time std')
    plt.legend()

    plt.show()
