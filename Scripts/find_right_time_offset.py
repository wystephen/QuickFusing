# -*- coding:utf-8 -*-
# Create by steve in 17-5-8 at 下午10:16


import os

if __name__ == '__main__':

    for time_offset in range(-30,10,1):
        os.system("../cmake-build-debug/dw {0} 0.001 0.001 10".format(time_offset))