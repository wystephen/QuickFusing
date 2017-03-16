import numpy as np
import scipy as sp

import matplotlib.pyplot as plt
import os

from array import array

if __name__ == '__main__':
    dir_name = '/home/steve/locate/RunResult/'
    all_result_array = array('d')
    # print(os.listdir(dir_name))
    for d1 in os.listdir(dir_name):
        if len(d1) == 1:
            print(dir_name + d1 + '/')
            print(int(d1))
            if d1 not in ['5']:
                continue
            uwb_real_pose = np.loadtxt('/home/steve/locate/{0}RealPose.csv'.format(int(d1)), delimiter=',')
            for para in os.listdir(dir_name + d1 + '/'):
                particle_num = int(para.split('-')[0])
                t_sigma = float(para.split('-')[1])
                eval_sigma = float(para.split('-')[2])
                # print(particle_num,t_sigma,eval_sigma)
                max_index = 0
                data_length = 0
                for file_name in os.listdir(dir_name + d1 + '/' + para + '/'):

                    if 'txt' in file_name:
                        # print(file_name)
                        if int(file_name.split('-')[0]) > max_index:
                            max_index = int(file_name.split('-')[0])
                    if 'uwb.txt' in file_name:
                        t = np.loadtxt(dir_name + d1 + '/' + para + '/' + file_name)
                        data_length = t.shape[0]
                        # print("max_index :" , max_index)


                        # load data
                uwb_err = np.zeros([max_index, data_length])
                fus_err = np.zeros([max_index, data_length])
                uwb_time = np.zeros([max_index])
                fus_time = np.zeros([max_index])

                for file_name in os.listdir(dir_name + d1 + '/' + para + '/'):
                    this_dir = dir_name + d1 + '/' + para + '/'
                    if 'txt' in file_name:
                        index = int(file_name.split('-')[0])
                        # print('index :',index-1)
                        if 'fus.txt' in file_name:
                            tmp_uwb = np.loadtxt(this_dir + file_name)
                            fus_err[index - 1, :] = np.sum((tmp_uwb - uwb_real_pose[:, 0:2]) ** 2.0, 1) ** 0.5
                            continue
                        if 'uwb.txt' in file_name:
                            tmp_uwb = np.loadtxt(this_dir + file_name)
                            uwb_err[index - 1, :] = np.sum((tmp_uwb - uwb_real_pose[:, 0:2]) ** 2.0, 1) ** 0.5
                            continue
                        if 'fustime.txt' in file_name:
                            tmp_time = np.loadtxt(this_dir + file_name)
                            fus_time[index - 1] = tmp_time
                            # if particle_num > 10000:
                            #     print(tmp_uwb)
                            #     print(fus_time)
                            continue
                        if 'uwbtime.txt' in file_name:
                            tmp_time = np.loadtxt(this_dir + file_name)
                            uwb_time[index - 1] = tmp_time
                            # print(tmp_time)
                            continue
                print("para:", particle_num, t_sigma, eval_sigma, "uwb err:", np.mean(uwb_err), "fuse err:",
                      np.mean(fus_err), "fuse std:", np.std(fus_err),
                      "uwb time:", np.mean(uwb_time), 'fus time :', np.mean(fus_time), 'fus time std:',
                      np.std(fus_time))
                all_result_array.append(particle_num)
                all_result_array.append(t_sigma)
                all_result_array.append(eval_sigma)
                all_result_array.append(np.mean(uwb_err))
                all_result_array.append(np.mean(fus_err))
                all_result_array.append(np.std(fus_err))
                all_result_array.append(np.mean(uwb_time))
                all_result_array.append(np.mean(fus_time))
                all_result_array.append(np.std(fus_time))

                all_result_np = np.frombuffer(all_result_array, dtype=np.float).reshape(-1, 9)
                np.savetxt('all_result.data', all_result_np)



                # Read
