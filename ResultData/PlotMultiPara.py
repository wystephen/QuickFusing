import numpy as np
import scipy as sp

import matplotlib.pyplot as plt

if __name__ == '__main__':
    all_result = np.loadtxt("5all_result.data")

    sort_order = np.argsort(all_result[:, 0])
    all_result = all_result[sort_order, :]
    print(all_result)
    particle_num_list = list()
    t_sigma_list = list()
    eval_sigma_list = list()

    for i in range(all_result.shape[0]):
        t_p = all_result[i, 0]
        t_t = all_result[i, 1]
        t_e = all_result[i, 2]
        # for j in range(len(particle_num_list))
        # bool_p=False
        # bool_t=False
        # bool_e=False
        if t_p not in particle_num_list:
            particle_num_list.append(t_p)
        if t_t not in t_sigma_list:
            t_sigma_list.append(t_t)
        if t_e not in eval_sigma_list:
            eval_sigma_list.append(t_e)

    for t_p in particle_num_list:
        for t_t in t_sigma_list:
            for t_e in eval_sigma_list:
                select_list = list()
                for i in range(all_result.shape[0]):
                    if np.abs(all_result[i, 1] - t_t) < 1e-5 and np.abs(all_result[i, 2] - t_e) < 1e-5:
                        select_list.append(i)

                plt.figure(1)
                plt.clf()
                plt.grid(True)

                # select_list = select_list[0:11]
                plt.plot(all_result[select_list, 0], all_result[select_list, 4], 'r-', label='Fusing')
                # plt.plot(all_result[select_list, 0], all_result[select_list, 4] + all_result[select_list, 5], 'r-.')
                # plt.plot(all_result[select_list, 0], all_result[select_list, 4] - all_result[select_list, 5], 'r-.')

                # plt.plot(all_result[select_list,0],all_result[select_list,5],'r',label = 'fus std')
                plt.plot(all_result[select_list, 0], all_result[select_list, 3], 'b-', label='Only-UWB')
                plt.xlabel('Particle number')
                plt.ylabel('Average Error/m')
                plt.legend()
                plt.savefig("{0}-{1}-1.jpg".format(str(t_t), str(t_e)))

                plt.figure(2)
                plt.clf()
                plt.grid(True)
                plt.plot(all_result[select_list, 0], all_result[select_list, 7], 'r-', label='Fusing')
                # plt.plot(all_result[select_list, 0], all_result[select_list, 7] + all_result[select_list, 8], 'r-.')
                # plt.plot(all_result[select_list, 0], all_result[select_list, 7] - all_result[select_list, 8], 'r-.')
                # plt.plot(all_result[select_list, 0], all_result[select_list, 8], 'b-.', label='fuse time std')
                plt.plot(all_result[select_list, 0], all_result[select_list, 6], 'b-', label='Only-UWB')
                plt.xlabel('Particle number')
                plt.ylabel('Average Time Waste/s')
                plt.legend()

                plt.savefig("{0}-{1}-2.jpg".format(t_t, t_e))

                # plt.show()
