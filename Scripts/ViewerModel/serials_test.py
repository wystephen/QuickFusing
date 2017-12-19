# -*- coding:utf-8 -*-
# carete by steve at  2016 / 11 / 01　14:34

import serial

t = serial.Serial('com12',9600)
print t.portstr