#!/usr/bin/env python
# coding=utf-8


import os
#import numpy as np

for x in [1000,3000,6000,10000,20000,40000]:
    for y in [0.2,0.4,0.5,0.6,0.8,1.0,1.4,1.8,2.2,2.8,3.2]:
        for z in [0.2,0.4,0.8,0.9,1.2,1.5,2.0,2.2,2.4]:
            for k in range(4):
                os.system('./cmake-build-debug/QuickFusing '+str(x) + ' ' + str(y) + ' ' +str(z))

