# Created by steve on 18-1-9 下午4:49
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
import scipy as sp
import matplotlib.pyplot as plt

import re


class UwbProcess:
    def __init__(self, name, mac_file_name):
        self.file_name = name

        file = open(name)

        lines = file.readlines()

        mac_list = list()
        mac_file_lines = open(mac_file_name).readlines()
        beaconSet = np.zeros([len(mac_file_lines), 3])
        for mac_line in mac_file_lines:
            mac_list.append(mac_line.split(',')[0])
            for k in range(3):
                beaconSet[len(mac_list) - 1, k] = mac_line.split(',')[k + 1]

        print(mac_list)
        print(beaconSet)

        m_re = re.compile('\\{[0-9|A-Z]{8}:[\\.|0-9]{1,},[\\.|0-9]{1,},}')
        # for
        uwb_data = np.zeros([len(lines), len(mac_list) + 1])
        uwb_data = uwb_data - 10.0
        for i in range(len(lines)):
            the_line = lines[i]
            uwb_data[i, 0] = float(the_line.split(',')[1])

            # print(m_re.findall(the_line))
            for m in m_re.findall(the_line):
                mac = m[1:m.find(':')]
                dis = m[m.find(':') + 1:m.find(',')]
                print('mac:', mac, ' dis:', dis, 'index:', mac_list.index(mac))
                uwb_data[i, 1 + mac_list.index(mac)] = dis

        plt.figure()
        for i in range(1, uwb_data.shape[1]):
            if (np.max(uwb_data[:, i]) > 0):
                plt.plot(uwb_data[:, 0], uwb_data[:, i], '.', label=str(i))
        plt.legend()
        plt.grid()
        self.uwb_data = uwb_data
        self.beaconSet = beaconSet
        # plt.show()

        #


if __name__ == '__main__':
    uwb_file_name = UwbProcess("/home/steve/Data/FusingLocationData/0013/HEAD_UWB.data",
                               '/home/steve/Data/FusingLocationData/mac.txt')
