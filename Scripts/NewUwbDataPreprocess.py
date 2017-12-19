# Created by steve on 17-12-18 下午5:33
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

import os
import re
import numpy as np
import matplotlib.pyplot as plt
import array


class NewUwbDataPre:

    def __init__(self, data_file_dir, mac_pose_file_name):
        '''

        :param data_file_dir: directory of data file.

        :param mac_pose_file_name: i.e. /dir/to/file/file_name.csv
        '''

        # print(self.__class__)
        ### Read mac pose
        mac_pose_file = open(mac_pose_file_name)
        self.mac_dic = dict()
        for index, line in enumerate(mac_pose_file.readlines()):
            self.mac_dic[line.split(',')[0]] = \
                (int(index),
                 float(line.split(',')[1]),
                 -float(line.split(',')[2]))

        max_index = 0
        for key in self.mac_dic:
            print(key,
                  self.mac_dic.get(key)[0],
                  self.mac_dic.get(key)[1],
                  self.mac_dic.get(key)[2])
            max_index = max(max_index, self.mac_dic.get(key)[0])
        print('max index :', max_index)

        ### generate new data
        self.data_file_name = ''
        if not data_file_dir[-1] is '/':
            data_file_dir = data_file_dir + '/'
        for name in os.listdir(data_file_dir):
            if 'uwbdata.txt' in name:
                self.data_file_name = data_file_dir + name
                self.start_time = float(name.split('_')[0])
        print('data file name:', self.data_file_name)

        self.tmp_array = array.array('d')

        all_lines = open(self.data_file_name).readlines()
        self.first_time = float(all_lines[0].split('[')[1].split(']')[0])

        local_time = 0.0
        the_index = 0
        # tmp_range = list()
        tmp_range = np.zeros(max_index + 1)
        tmp_range -= 10.0

        while the_index < len(all_lines):
            # for each line
            if 'F1' in all_lines[the_index]:
                ### out put
                self.tmp_array.append(local_time - self.first_time + self.start_time)
                for i in range(tmp_range.shape[0]):
                    self.tmp_array.append(float(tmp_range[i]))
                    tmp_range[i] = -10.0

            else:
                local_time = float(all_lines[the_index].split('[')[1].split(']')[0])
                mac_str = all_lines[the_index].split(' ')[4]
                mac_range = float(all_lines[the_index].split(' ')[5])
                tmp_range[self.mac_dic.get(mac_str)[0]] = mac_range

            the_index += 1

        self.uwb_data = np.frombuffer(self.tmp_array, dtype=np.float).reshape([-1, max_index + 2])

        self.beaconset = np.zeros([max_index + 1, 3])
        for key in self.mac_dic:
            self.beaconset[self.mac_dic.get(key)[0], 0] = self.mac_dic.get(key)[1]
            self.beaconset[self.mac_dic.get(key)[0], 1] = self.mac_dic.get(key)[2]

    def show(self):
        plt.figure()
        for i in range(1, self.uwb_data.shape[1]):
            plt.plot(self.uwb_data[:, 0], self.uwb_data[:, i], '+', label=str(i))
        plt.grid()
        plt.legend()
        # plt.show()

        plt.figure()
        plt.plot(self.beaconset[:, 0], self.beaconset[:, 1], 'r*')
        for i in range(self.beaconset.shape[0]):
            plt.text(self.beaconset[i, 0], self.beaconset[i, 1], str(i))
        plt.grid()


if __name__ == '__main__':
    dir_name = '/home/steve/Data/NewIU/'
    num_data = 17


    nudp = NewUwbDataPre(dir_name + str(num_data) + '/', dir_name + 'MacPose.csv')
    nudp.show()
    plt.show()
