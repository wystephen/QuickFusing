# -*- coding:utf-8 -*-
# Create by steve in 17-5-8 at 下午10:16


import os

if __name__ == '__main__':

    for time_offset in range(68,77,1):
        os.system("../cmake-build-debug/dw {0}".format(time_offset))