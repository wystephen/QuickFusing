#!/usr/bin/env python
# coding=utf-8

import os
import numpy as np

dir_name = 'tmp_file_dir'

for file_name in os.listdir(dir_name):
    if not 'csv' in file_name:
        tmp = np.loadtxt(dir_name + '/' + file_name)
        np.savetxt(dir_name + '/' + file_name + '.csv',tmp,delimiter=',')
        
        
