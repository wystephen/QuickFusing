#! /usr/bin/env python
# -*- coding: utf-8 -*-
# --- create by :UwbDataPreprocess at : 17-5-7  下午5:09

import os

import re

import numpy as np

import matplotlib.pyplot as plt
import array


class UwbDataPre:
    def __init__(self, dir_name):
        self.dir_name = dir_name
        self.file_name = dir_name

        self.start_time = 0.0

        self.tmp_array = array.array('d')

        for file in os.listdir(dir_name):
            print(file)
            if 'uwbdata.txt' in file:
                self.file_name += file
                self.start_time = float(file.split('_')[0])

        print("filr name is :", self.file_name)
        print("start_time :", self.start_time)

        this_file = open(self.file_name)

        is_new_ob = False

        mac_list = list()

        for line in this_file.readlines():
            # print(line)
            if line.split(' ')[2] == '@R':
                # print(line)
                if line.split(' ')[3] == '1' or line.split(' ')[3] == '0':
                    mac_name = line.split(' ')[4]
                    if mac_name not in mac_list:
                        mac_list.append(mac_name)

        print("mac list :", mac_list)
        this_file.close()
        this_file = open(self.file_name)

        is_first_line = True

        for line in this_file.readlines():
            print("time:", line.split('[')[1].split(']')[0])
            time_in_the_line = float(line.split('[')[1].split(']')[0])

            if is_first_line:
                self.start_time -= time_in_the_line
                print("start time :", time_in_the_line)
                is_first_line = False

            if line.split(' ')[2] == '@R':
                print(line)
                if line.split(' ')[3] != 'F1':
                    print(line)
                    mac_name = line.split(' ')[4]
                    self.tmp_array.append(float(self.start_time + time_in_the_line))
                    print("time : ", float(self.start_time + time_in_the_line))
                    for i in range(len(mac_list)):

                        if mac_name != mac_list[i]:
                            self.tmp_array.append(-10.0)
                        else:
                            self.tmp_array.append(float(line.split(' ')[5]))

        print('len mac list:', len(mac_list))
        self.result_uwb = np.frombuffer(self.tmp_array, dtype=np.float).reshape([-1, len(mac_list) + 1])

        print(self.result_uwb)

        # self.result_uwb[150:320,1:] -=100.0
        # self.result_uwb[:,1] -=100.0
        # self.result_uwb[:,2] -=100.0
        # self.result_uwb[:,4]  -= 100.0

    def save(self):
        np.savetxt(self.dir_name + "uwb_result.csv", self.result_uwb, '%.4f', delimiter=',')

    '''
    
    Show result
    '''

    def show(self):
        '''

        :return:
        '''
        plt.figure()
        plt.plot(self.result_uwb[:, 0], 'r')

        plt.figure()
        plt.title('uwb ')
        for i in range(1, self.result_uwb.shape[1]):
            # if i != 4 :
            #     continue
            plt.plot(self.result_uwb[:, i], '*', label='i:' + str(i))
        plt.legend()
        plt.show()

        # self.start_time

    def filter(self):
        print("try to filter")

        # filter for each Beacon
        for i in range(self.result_uwb.shape[1]):
            index_list = list()
            # find all valid data
            for j in range(self.result_uwb.shape[0]):
                if self.result_uwb[j, i] > -0.1:
                    index_list.append(j)

            # Main filter process
            step_len = 5
            tmp_value_list = list()
            for index in range(step_len, len(index_list) - step_len):
                if abs((self.result_uwb[index_list[index - step_len], i] +
                        self.result_uwb[index_list[index + step_len], i])
                       - 2.0 * self.result_uwb[index_list[index], i]) > 1.92:
                    # print(abs((self.result_uwb[index_list[index-step_len],i]+
                    #         self.result_uwb[index_list[index+step_len],i])
                    #    -2.0* self.result_uwb[index_list[index],i]) )
                    tmp_value_list.append(-10.0)
                else:
                    tmp_value_list.append(self.result_uwb[index_list[index], i])

            for index in range(step_len, len(index_list) - step_len):
                print(index - step_len, 'of', len(tmp_value_list))
                self.result_uwb[index_list[index], i] = \
                    tmp_value_list[index - step_len]


if __name__ == '__main__':
    # udp = UwbDataPre("/home/steve/Data/NewRecord/Record2/")
    # udp = UwbDataPre("/home/steve/tmp/test/44/")
    # udp = UwbDataPre("/home/steve/Code/Mini_IMU/Scripts/IMUWB/92/")
    udp = UwbDataPre("/home/steve/Code/Mini_IMU/Scripts/IMUWB/92/")
    # udp.filter()

    udp.save()
    udp.show()
